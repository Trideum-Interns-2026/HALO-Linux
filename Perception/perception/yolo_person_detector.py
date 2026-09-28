"""Subscribes to a camera image topic and detects a person with a YOLO
model (Ultralytics YOLOv8n by default), filtered to the COCO "person" class.

Same publish contract as person_detector.py (HOG-based), so either can be
picked per launch file and the tracker consumes both the same way: centroid
+ bbox height on ~/person/position (PointStamped: x=u, y=v,
z=bbox_height_px), a distance estimate (meters) on ~/person/distance, and an
annotated debug image on ~/person/debug_image — the latter only while
something is subscribed to it, since encoding a 1080p frame isn't free.

Runs on the NVIDIA GPU when PyTorch can see one, CPU otherwise.
"""

import threading

import cv2
import rclpy
import torch
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32
from ultralytics import YOLO

COCO_PERSON_CLASS = 0


class YoloPersonDetector(Node):
    """Runs frame capture and YOLO inference on separate threads.

    The ROS callback (on_image) only does the cheap cv_bridge decode and
    hands the frame off — it never runs inference itself, so it's never
    blocked waiting on a slow YOLO pass. A background thread takes whatever
    the newest handed-off frame is and runs detection + publishing. The
    handoff is a single slot, not a queue: a frame that arrives mid-inference
    replaces the pending one, so the worker always processes the latest
    frame instead of working through a stale backlog.
    """

    def __init__(self):
        super().__init__("yolo_person_detector")

        self.declare_parameter("image_topic", "/camera/front/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/front/camera_info")
        # Baked into the image by the Dockerfile, so no download at runtime.
        self.declare_parameter("model_path", "/opt/yolo/yolov8n.pt")
        self.declare_parameter("min_confidence", 0.5)
        # YOLO resizes frames to this before inference. Smaller = faster but
        # loses people who are far away.
        self.declare_parameter("inference_size", 640)
        # "" picks the GPU when PyTorch sees one; "cpu" or "cuda:0" to force.
        self.declare_parameter("device", "")
        # Real-world height of the target person, in meters — matches
        # Perception/models/standing_person.sdf (~1.9m). YOLO's boxes fit the
        # person tightly, so unlike HOG no margin correction is needed.
        self.declare_parameter("person_height_m", 1.9)
        self.declare_parameter("publish_debug_image", True)

        image_topic = self.get_parameter("image_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.inference_size = int(self.get_parameter("inference_size").value)
        self.person_height_m = float(self.get_parameter("person_height_m").value)
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)
        self.device = self.get_parameter("device").value or (
            "cuda:0" if torch.cuda.is_available() else "cpu"
        )

        self.bridge = CvBridge()
        self.focal_length_px = None  # populated once camera_info arrives
        self.model = YOLO(self.get_parameter("model_path").value)

        # Only the newest frame matters for a live tracker: BEST_EFFORT +
        # depth 1 drops stale frames at the DDS layer instead of queueing
        # them, on top of the single-slot handoff below.
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.image_sub = self.create_subscription(Image, image_topic, self.on_image, image_qos)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self.on_camera_info, 10
        )
        self.position_pub = self.create_publisher(PointStamped, "~/person/position", 10)
        self.distance_pub = self.create_publisher(Float32, "~/person/distance", 10)
        self.debug_image_pub = (
            self.create_publisher(Image, "~/person/debug_image", 10)
            if self.publish_debug_image
            else None
        )

        # Single-slot handoff between the ROS callback thread (producer) and
        # the inference worker thread (consumer). The condition guards both
        # the slot and the shutdown flag.
        self._cond = threading.Condition()
        self._pending = None  # (frame, header) or None
        self._shutdown = False
        self._worker = threading.Thread(target=self._inference_loop, daemon=True)
        self._worker.start()

        self.get_logger().info(
            f"Subscribed to '{image_topic}' for YOLO person detection on {self.device}"
        )

    def on_camera_info(self, msg: CameraInfo):
        # K is the row-major 3x3 intrinsic matrix; K[4] is fy in pixels, the
        # vertical focal length that matches a height measurement.
        self.focal_length_px = msg.k[4]

    def on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().warn(f"cv_bridge conversion failed: {exc}")
            return

        with self._cond:
            self._pending = (frame, msg.header)  # replaces any unprocessed frame
            self._cond.notify()

    def _inference_loop(self):
        while True:
            with self._cond:
                while self._pending is None and not self._shutdown:
                    self._cond.wait()
                if self._shutdown:
                    return
                frame, header = self._pending
                self._pending = None

            self._process(frame, header)

    def destroy_node(self):
        with self._cond:
            self._shutdown = True
            self._cond.notify()
        self._worker.join(timeout=2.0)
        super().destroy_node()

    def _process(self, frame, header):
        detection = self._detect_person(frame)
        distance_m = None

        if detection is not None:
            (cx, cy), (_x1, y1, _x2, y2), _score = detection
            h = y2 - y1
            point = PointStamped()
            point.header = header
            point.point.x = float(cx)
            point.point.y = float(cy)
            point.point.z = float(h)
            self.position_pub.publish(point)

            if self.focal_length_px is not None and h > 0:
                # Pinhole model: apparent_height_px / focal_length_px ==
                # real_height_m / distance_m, solved for distance_m.
                distance_m = self.person_height_m * self.focal_length_px / h
                self.distance_pub.publish(Float32(data=float(distance_m)))

        if (
            self.debug_image_pub is not None
            and self.debug_image_pub.get_subscription_count() > 0
        ):
            self._publish_debug_image(frame, detection, distance_m, header)

    def _detect_person(self, frame):
        results = self.model.predict(
            frame,
            imgsz=self.inference_size,
            classes=[COCO_PERSON_CLASS],
            conf=self.min_confidence,
            device=self.device,
            verbose=False,
        )[0]
        if len(results.boxes) == 0:
            return None

        best = max(results.boxes, key=lambda b: float(b.conf[0]))
        x1, y1, x2, y2 = (float(v) for v in best.xyxy[0])
        return ((x1 + x2) / 2.0, (y1 + y2) / 2.0), (x1, y1, x2, y2), float(best.conf[0])

    def _publish_debug_image(self, frame, detection, distance_m, header):
        debug = frame.copy()
        if detection is not None:
            (cx, cy), (x1, y1, x2, y2), score = detection
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            cv2.rectangle(debug, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"person conf={score:.2f}"
            if distance_m is not None:
                label += f"  dist={distance_m:.2f}m"
            cv2.putText(
                debug,
                label,
                (x1, max(0, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
            cv2.circle(debug, (int(cx), int(cy)), 4, (0, 255, 0), -1)

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().warn(f"cv_bridge conversion failed for debug image: {exc}")
            return
        debug_msg.header = header
        self.debug_image_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloPersonDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
