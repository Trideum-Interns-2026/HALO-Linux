"""Subscribes to a camera image topic and detects a person with a YOLO
model (Ultralytics YOLOv8n by default), filtered to the COCO "person" class.

Same publish contract as person_detector.py (HOG-based), so either can be
picked per launch file and the tracker consumes both the same way: centroid
+ bbox height on ~/person/position (PointStamped: x=u, y=v,
z=bbox_height_px), a distance estimate (meters) on ~/person/distance, and an
annotated debug image (960px wide) on ~/person/debug_image — the latter
only while something is subscribed to it, since drawing and encoding it
isn't free. View it with `ros2 run perception debug_image_viewer`.

Runs on the NVIDIA GPU when PyTorch can see one, CPU otherwise.
"""

import threading
import time

import cv2
import rclpy
import torch
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
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
        # Debug images are shrunk to this width before publishing: a raw
        # 1080p frame is ~6MB, which is most of what makes rqt_image_view
        # lag. 0 publishes full resolution.
        self.declare_parameter("debug_image_width", 960)

        image_topic = self.get_parameter("image_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.inference_size = int(self.get_parameter("inference_size").value)
        self.person_height_m = float(self.get_parameter("person_height_m").value)
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)
        self.debug_image_width = int(self.get_parameter("debug_image_width").value)
        self.device = self.get_parameter("device").value or (
            "cuda:0" if torch.cuda.is_available() else "cpu"
        )

        self.bridge = CvBridge()
        self.focal_length_px = None  # populated once camera_info arrives
        self.model = YOLO(self.get_parameter("model_path").value)

        # Only the newest frame matters for a live tracker, so depth 1 — but
        # RELIABLE, not BEST_EFFORT. A 1080p frame is ~6MB, which Fast DDS
        # splits into ~100 fragments; best-effort loses the whole frame if
        # any one fragment is dropped, and measured in the sim that cut a
        # 15 Hz camera to ~2 Hz with multi-second gaps. Reliable repairs lost
        # fragments, and KEEP_LAST 1 still means a slow reader only ever
        # gets the newest frame, never a backlog. The same profile is used
        # for the debug image publishers, for the same reasons.
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
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
            self.create_publisher(Image, "~/person/debug_image", image_qos)
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

        # Pipeline stats, logged every few seconds so a slow stage shows up
        # in the node's output: frames received from the camera, frames
        # replaced in the slot before the worker got to them, frames
        # processed, and time spent in inference / debug-image publishing.
        # Guarded by self._cond.
        self._stats = self._empty_stats()
        self.create_timer(5.0, self._log_stats)

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
            self._stats["received"] += 1
            if self._pending is not None:
                self._stats["dropped"] += 1
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

            try:
                self._process(frame, header)
            except Exception:
                if not self.context.ok():
                    return  # Ctrl+C shut ROS down mid-frame; nothing to publish to
                raise

    @staticmethod
    def _empty_stats():
        return {"received": 0, "dropped": 0, "processed": 0, "infer_s": 0.0,
                "infer_max_s": 0.0, "debug_s": 0.0, "since": time.monotonic()}

    def _log_stats(self):
        with self._cond:
            s, self._stats = self._stats, self._empty_stats()
        elapsed = time.monotonic() - s["since"]
        n = max(s["processed"], 1)
        self.get_logger().info(
            f"camera in {s['received'] / elapsed:.1f} Hz, "
            f"processed {s['processed'] / elapsed:.1f} Hz, "
            f"dropped {s['dropped']} stale | "
            f"inference avg {1000 * s['infer_s'] / n:.0f} ms "
            f"(max {1000 * s['infer_max_s']:.0f}), "
            f"debug image avg {1000 * s['debug_s'] / n:.0f} ms"
        )

    def destroy_node(self):
        with self._cond:
            self._shutdown = True
            self._cond.notify()
        self._worker.join(timeout=2.0)
        super().destroy_node()

    def _process(self, frame, header):
        t0 = time.monotonic()
        detection = self._detect_person(frame)
        t_infer = time.monotonic() - t0
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

        with self._cond:
            self._stats["processed"] += 1
            self._stats["infer_s"] += t_infer
            self._stats["infer_max_s"] = max(self._stats["infer_max_s"], t_infer)
            self._stats["debug_s"] += time.monotonic() - t0 - t_infer

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

    def _shrink_for_debug(self, image):
        if self.debug_image_width <= 0 or image.shape[1] <= self.debug_image_width:
            return image
        scale = self.debug_image_width / image.shape[1]
        return cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)

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
            debug_msg = self.bridge.cv2_to_imgmsg(
                self._shrink_for_debug(debug), encoding="bgr8"
            )
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
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Ctrl+C: rclpy's signal handler has already shut ROS down
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
