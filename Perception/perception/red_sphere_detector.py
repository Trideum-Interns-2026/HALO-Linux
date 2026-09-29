"""Subscribes to a camera image topic and detects a red sphere with OpenCV.

Publishes the detected sphere's pixel centroid + apparent radius on
~/red_sphere/position (as a PointStamped: x=u, y=v, z=radius_px) whenever a
detection above the minimum area passes, plus a distance estimate (in
meters) on ~/red_sphere/distance derived from the known real-world sphere
size and the camera's focal length, and an annotated debug image on
~/red_sphere/debug_image on every frame.
"""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32

# Red wraps around hue 0, so two ranges are combined into one mask.
DEFAULT_LOWER_RED_1 = (0, 120, 70)
DEFAULT_UPPER_RED_1 = (10, 255, 255)
DEFAULT_LOWER_RED_2 = (170, 120, 70)
DEFAULT_UPPER_RED_2 = (180, 255, 255)


class RedSphereDetector(Node):
    def __init__(self):
        super().__init__("red_sphere_detector")

        # The gazebo_ros camera plugin publishes under
        # <namespace>/<camera_name>/image_raw — with namespace "/camera" and
        # camera_name "front" that's /camera/front/image_raw, not
        # /camera/image_raw.
        self.declare_parameter("image_topic", "/camera/front/image_raw")
        # Defaults to the image topic's sibling camera_info topic (same
        # namespace/camera_name, just image_raw -> camera_info).
        self.declare_parameter("camera_info_topic", "/camera/front/camera_info")
        self.declare_parameter("min_contour_area", 150.0)
        self.declare_parameter("publish_debug_image", True)
        # Debug images are shrunk to this width before publishing: a raw
        # 1080p frame is ~6MB, which is most of what makes rqt_image_view
        # lag. 0 publishes full resolution.
        self.declare_parameter("debug_image_width", 960)
        # Real-world diameter of the target sphere, in meters — matches
        # Perception/models/red_sphere.sdf (radius 0.2m). Used with the
        # camera's focal length to estimate distance from apparent size.
        self.declare_parameter("sphere_diameter_m", 0.4)

        image_topic = self.get_parameter("image_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        self.min_contour_area = float(self.get_parameter("min_contour_area").value)
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)
        self.debug_image_width = int(self.get_parameter("debug_image_width").value)
        self.sphere_diameter_m = float(self.get_parameter("sphere_diameter_m").value)

        self.bridge = CvBridge()
        self.focal_length_px = None  # populated once camera_info arrives

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
        self.image_sub = self.create_subscription(
            Image, image_topic, self.on_image, image_qos
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self.on_camera_info, 10
        )
        self.position_pub = self.create_publisher(
            PointStamped, "~/red_sphere/position", 10
        )
        self.distance_pub = self.create_publisher(
            Float32, "~/red_sphere/distance", 10
        )
        self.debug_image_pub = (
            self.create_publisher(Image, "~/red_sphere/debug_image", image_qos)
            if self.publish_debug_image
            else None
        )
        # Raw HSV mask, published whether or not anything is detected — the
        # fastest way to check if the color threshold is even seeing the
        # sphere's red before worrying about contour/bbox logic.
        self.mask_debug_pub = (
            self.create_publisher(Image, "~/red_sphere/mask_debug", image_qos)
            if self.publish_debug_image
            else None
        )

        self.get_logger().info(f"Subscribed to '{image_topic}' for red sphere detection")

    def on_camera_info(self, msg: CameraInfo):
        # K is the row-major 3x3 intrinsic matrix; K[0] is fx in pixels.
        self.focal_length_px = msg.k[0]

    def on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().warn(f"cv_bridge conversion failed: {exc}")
            return

        detection, mask = self._detect_red_sphere(frame)
        distance_m = None

        if detection is not None:
            (cx, cy), radius, _bbox = detection
            point = PointStamped()
            point.header = msg.header
            point.point.x = float(cx)
            point.point.y = float(cy)
            point.point.z = float(radius)
            self.position_pub.publish(point)

            if self.focal_length_px is not None and radius > 0:
                # Pinhole model: apparent_size_px / focal_length_px ==
                # real_size_m / distance_m, solved for distance_m.
                distance_m = (
                    self.sphere_diameter_m * self.focal_length_px
                ) / (2.0 * radius)
                self.distance_pub.publish(Float32(data=float(distance_m)))

        # Encoding 1080p debug frames isn't free — skip them unless something
        # (e.g. rqt_image_view) is actually watching.
        if (
            self.debug_image_pub is not None
            and self.debug_image_pub.get_subscription_count() > 0
        ):
            self._publish_debug_image(frame, detection, distance_m, msg.header)
        if (
            self.mask_debug_pub is not None
            and self.mask_debug_pub.get_subscription_count() > 0
        ):
            self._publish_mask_debug(mask, msg.header)

    def _detect_red_sphere(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, DEFAULT_LOWER_RED_1, DEFAULT_UPPER_RED_1)
        mask2 = cv2.inRange(hsv, DEFAULT_LOWER_RED_2, DEFAULT_UPPER_RED_2)
        mask = cv2.bitwise_or(mask1, mask2)

        # Clean up small speckle noise before contour detection.
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, mask

        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < self.min_contour_area:
            return None, mask

        (cx, cy), radius = cv2.minEnclosingCircle(largest)
        bbox = cv2.boundingRect(largest)  # (x, y, w, h)
        return ((cx, cy), radius, bbox), mask

    def _shrink_for_debug(self, image):
        if self.debug_image_width <= 0 or image.shape[1] <= self.debug_image_width:
            return image
        scale = self.debug_image_width / image.shape[1]
        return cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)

    def _publish_debug_image(self, frame, detection, distance_m, header):
        debug = frame.copy()
        if detection is not None:
            (cx, cy), radius, (x, y, w, h) = detection
            cv2.rectangle(debug, (x, y), (x + w, y + h), (0, 255, 0), 2)
            label = f"red_sphere r={radius:.0f}px"
            if distance_m is not None:
                label += f"  dist={distance_m:.2f}m"
            cv2.putText(
                debug,
                label,
                (x, max(0, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )
            cv2.circle(debug, (int(cx), int(cy)), 3, (0, 255, 0), -1)

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(
                self._shrink_for_debug(debug), encoding="bgr8"
            )
        except CvBridgeError as exc:
            self.get_logger().warn(f"cv_bridge conversion failed for debug image: {exc}")
            return
        debug_msg.header = header
        self.debug_image_pub.publish(debug_msg)

    def _publish_mask_debug(self, mask, header):
        try:
            mask_msg = self.bridge.cv2_to_imgmsg(
                self._shrink_for_debug(mask), encoding="mono8"
            )
        except CvBridgeError as exc:
            self.get_logger().warn(f"cv_bridge conversion failed for mask debug: {exc}")
            return
        mask_msg.header = header
        self.mask_debug_pub.publish(mask_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RedSphereDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()