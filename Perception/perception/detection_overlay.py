"""Detection overlay node: draws the person box on the camera image, for watching the sim.

A debug view, not part of the detection pipeline: it only reads the camera image and the
detector's output and publishes a smaller picture with the box drawn on it, so a viewer such as
rqt_image_view shows what Perception sees.

Topics
    in   /camera/front/image_raw   sensor_msgs/Image (reliable, depth 1; see image_reliability)
    in   /perception/detection    perception_msgs/Detection
    out  /perception/debug_image   sensor_msgs/Image (bgr8, scaled, header copied from the input)

Parameters (all optional, see declare_parameter below for defaults)
    image_topic, detection_topic, output_topic, image_reliability, scale, max_age_s
"""

import math

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from perception_msgs.msg import Detection
from rclpy.node import Node
from sensor_msgs.msg import Image

from perception.image_qos import IMAGE_RELIABILITY, image_qos

IMAGE_TOPIC = "/camera/front/image_raw"        # default input (sim camera bridge)
DETECTION_TOPIC = "/perception/detection"      # default detection input
OUTPUT_TOPIC = "/perception/debug_image"       # default output
DEFAULT_SCALE = 0.5      # the 1080p frame is shrunk so the viewer stays light
MIN_SCALE = 0.05
MAX_SCALE = 1.0
DEFAULT_MAX_AGE_S = 0.5  # a box older than this (vs the image stamp) is not drawn
BOX_COLOR_BGR = (0, 255, 0)
LABEL_COLOR_BGR = (0, 0, 0)
BOX_THICKNESS_PX = 2
FONT_SCALE = 0.6
LABEL_PAD_PX = 4
COORD_LIMIT_PX = 100000  # keeps absurd box values inside what OpenCV can draw


def sanitize_scale(value):
    """Output scale factor within [MIN_SCALE, MAX_SCALE]; non-finite falls back to the default."""
    try:
        scale = float(value)
    except (TypeError, ValueError):
        return DEFAULT_SCALE
    if not math.isfinite(scale):
        return DEFAULT_SCALE
    return min(max(scale, MIN_SCALE), MAX_SCALE)


def stamp_to_s(stamp):
    """A builtin_interfaces/Time as seconds."""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def is_fresh(detection_stamp_s, image_stamp_s, max_age_s):
    """True when the detection and the image are at most max_age_s apart (either order).

    The detector copies the image header, and its answer arrives a frame or two after the image,
    so the newest box is drawn on the newest image. Exactly at the limit still counts as fresh.
    """
    gap = abs(image_stamp_s - detection_stamp_s)
    return math.isfinite(gap) and math.isfinite(max_age_s) and gap <= max_age_s


def scale_box(box, scale):
    """Box (x_min, y_min, x_max, y_max) in pixels to integer pixels of the scaled image."""
    return tuple(round(min(max(v * scale, -COORD_LIMIT_PX), COORD_LIMIT_PX)) for v in box)


def draw_detection(frame, box, confidence):
    """Draw the box and a 'person 0.95' label on `frame` (in place) and return it.

    `box` is in the pixels of `frame`. A box partly or wholly outside the frame is clipped by
    OpenCV and never raises. The label sits above the box, or inside its top edge when the box
    touches the top of the image.
    """
    x_min, y_min, x_max, y_max = box
    cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), BOX_COLOR_BGR, BOX_THICKNESS_PX)
    label = f"person {confidence:.2f}"
    (text_w, text_h), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, FONT_SCALE, 1)
    top = y_min - text_h - baseline - LABEL_PAD_PX
    if top < 0:
        top = y_min + BOX_THICKNESS_PX
    bottom = top + text_h + baseline + LABEL_PAD_PX
    cv2.rectangle(frame, (x_min, top), (x_min + text_w + LABEL_PAD_PX, bottom),
                  BOX_COLOR_BGR, cv2.FILLED)
    cv2.putText(frame, label, (x_min + LABEL_PAD_PX // 2, bottom - baseline - LABEL_PAD_PX // 2),
                cv2.FONT_HERSHEY_SIMPLEX, FONT_SCALE, LABEL_COLOR_BGR, 1, cv2.LINE_AA)
    return frame


class DetectionOverlay(Node):
    """ROS2 node: camera frames and detections in, a picture with the box drawn on it out."""

    def __init__(self):
        super().__init__("detection_overlay")

        self.image_topic = self.declare_parameter("image_topic", IMAGE_TOPIC).value
        self.detection_topic = self.declare_parameter("detection_topic", DETECTION_TOPIC).value
        self.output_topic = self.declare_parameter("output_topic", OUTPUT_TOPIC).value
        self.image_reliability = self.declare_parameter(
            "image_reliability", IMAGE_RELIABILITY).value
        self.scale = sanitize_scale(self.declare_parameter("scale", DEFAULT_SCALE).value)
        self.max_age_s = float(self.declare_parameter("max_age_s", DEFAULT_MAX_AGE_S).value)

        self.bridge = CvBridge()
        self._latest = None   # (stamp_s, box, confidence) of the newest detection

        self.sub_image = self.create_subscription(
            Image, self.image_topic, self.on_image, image_qos(self.image_reliability))
        self.sub_detection = self.create_subscription(
            Detection, self.detection_topic, self.on_detection, 10)
        self.pub = self.create_publisher(Image, self.output_topic, 10)

        self.get_logger().info(
            f"Ready: image={self.image_topic}({self.image_reliability}) "
            f"detection={self.detection_topic} output={self.output_topic} "
            f"scale={self.scale} max_age={self.max_age_s}s")

    def on_detection(self, msg):
        """Remember the newest detection; the next image draws it."""
        box = (float(msg.x_min), float(msg.y_min), float(msg.x_max), float(msg.y_max))
        if not all(math.isfinite(v) for v in box):
            return
        self._latest = (stamp_to_s(msg.header.stamp), box, float(msg.confidence))

    def on_image(self, msg):
        """Shrink one camera frame, draw the newest fresh box on it and publish it."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except (CvBridgeError, ValueError, cv2.error) as e:   # bad image: skip, never crash
            self.get_logger().warn(
                f"Cannot convert the image, skipping frame: {e}", throttle_duration_sec=5.0)
            return
        if self.scale < 1.0:
            frame = cv2.resize(frame, None, fx=self.scale, fy=self.scale,
                               interpolation=cv2.INTER_AREA)
        else:
            frame = frame.copy()
        if self._latest is not None:
            stamp_s, box, confidence = self._latest
            if is_fresh(stamp_s, stamp_to_s(msg.header.stamp), self.max_age_s):
                draw_detection(frame, scale_box(box, self.scale), confidence)
        out = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        out.header = msg.header        # same timestamp and frame as the camera image
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DetectionOverlay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
