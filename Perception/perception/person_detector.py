"""Subscribes to a camera image topic and detects a standing person with
OpenCV's built-in HOG+SVM people detector — no marker or color needed.

Publishes the detected person's pixel centroid + bounding-box height on
~/person/position (as a PointStamped: x=u, y=v, z=bbox_height_px) whenever a
detection above the minimum confidence passes, plus a distance estimate (in
meters) on ~/person/distance derived from an assumed real-world person
height and the camera's focal length, and an annotated debug image on
~/person/debug_image on every processed frame.

Same topic shapes as red_sphere_detector, so the tracker can consume either
one by just pointing its position_topic/distance_topic parameters here.
"""

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32

# The default people detector was trained on 64x128 windows in which the
# person spans roughly the middle 96px of the height, so a detected box is
# ~1/0.75 taller than the person inside it.
HOG_BOX_PERSON_FRACTION = 0.75


class PersonDetector(Node):
    def __init__(self):
        super().__init__("person_detector")

        self.declare_parameter("image_topic", "/camera/front/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/front/camera_info")
        # Minimum SVM score for a detection to count. HOG throws weak false
        # positives on clutter; real people in sim usually score well above.
        self.declare_parameter("min_confidence", 0.5)
        # Frames are shrunk to this width before running HOG. The camera is
        # 1920x1080, and full-resolution HOG takes seconds per frame. Larger
        # = detects people further away (a person needs to be ~130px tall
        # after resizing) but slower.
        self.declare_parameter("detection_width", 960)
        # Real-world height of the target person, in meters — matches
        # Perception/models/standing_person.sdf (~1.9m). Used with the
        # camera's focal length to estimate distance from apparent height.
        self.declare_parameter("person_height_m", 1.9)
        self.declare_parameter("publish_debug_image", True)
        # Debug images are shrunk to this width before publishing: a raw
        # 1080p frame is ~6MB, which is most of what makes rqt_image_view
        # lag. 0 publishes full resolution.
        self.declare_parameter("debug_image_width", 960)

        image_topic = self.get_parameter("image_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.detection_width = int(self.get_parameter("detection_width").value)
        self.person_height_m = float(self.get_parameter("person_height_m").value)
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)
        self.debug_image_width = int(self.get_parameter("debug_image_width").value)

        self.bridge = CvBridge()
        self.focal_length_px = None  # populated once camera_info arrives
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

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

        self.get_logger().info(f"Subscribed to '{image_topic}' for person detection")

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

        detection = self._detect_person(frame)
        distance_m = None

        if detection is not None:
            (cx, cy), (_x, _y, _w, h), _score = detection
            point = PointStamped()
            point.header = msg.header
            point.point.x = float(cx)
            point.point.y = float(cy)
            point.point.z = float(h)
            self.position_pub.publish(point)

            if self.focal_length_px is not None and h > 0:
                # Pinhole model: apparent_height_px / focal_length_px ==
                # real_height_m / distance_m, solved for distance_m.
                person_px = h * HOG_BOX_PERSON_FRACTION
                distance_m = self.person_height_m * self.focal_length_px / person_px
                self.distance_pub.publish(Float32(data=float(distance_m)))

        # Encoding a 1080p debug frame isn't free — skip it unless something
        # (e.g. rqt_image_view) is actually watching.
        if (
            self.debug_image_pub is not None
            and self.debug_image_pub.get_subscription_count() > 0
        ):
            self._publish_debug_image(frame, detection, distance_m, msg.header)

    def _detect_person(self, frame):
        scale = min(1.0, self.detection_width / frame.shape[1])
        small = cv2.resize(frame, None, fx=scale, fy=scale) if scale < 1.0 else frame

        boxes, weights = self.hog.detectMultiScale(
            small, winStride=(8, 8), padding=(8, 8), scale=1.05
        )
        if len(boxes) == 0:
            return None

        best = max(range(len(boxes)), key=lambda i: float(weights[i]))
        score = float(weights[best])
        if score < self.min_confidence:
            return None

        # Back to full-resolution pixel coordinates, which is what the
        # tracker's camera_info intrinsics refer to.
        x, y, w, h = (float(v) / scale for v in boxes[best])
        return (x + w / 2.0, y + h / 2.0), (x, y, w, h), score

    def _shrink_for_debug(self, image):
        if self.debug_image_width <= 0 or image.shape[1] <= self.debug_image_width:
            return image
        scale = self.debug_image_width / image.shape[1]
        return cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)

    def _publish_debug_image(self, frame, detection, distance_m, header):
        debug = frame.copy()
        if detection is not None:
            (cx, cy), (x, y, w, h), score = detection
            x, y, w, h = int(x), int(y), int(w), int(h)
            cv2.rectangle(debug, (x, y), (x + w, y + h), (0, 255, 0), 2)
            label = f"person score={score:.2f}"
            if distance_m is not None:
                label += f"  dist={distance_m:.2f}m"
            cv2.putText(
                debug,
                label,
                (x, max(0, y - 8)),
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
    node = PersonDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
