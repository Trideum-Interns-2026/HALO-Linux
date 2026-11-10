"""Debug viewer: shows the overlay's JPEG stream in an OpenCV window, for watching the sim.

rqt_image_view needs a transport plugin to read JPEG and was slow on the raw 1080p-derived image,
so this small node reads /perception/debug_image/compressed directly. Decoding happens in the
callback, drawing happens on the main thread, and only the newest frame is ever shown, so a slow
display skips frames instead of lagging behind.

Topics
    in   /perception/debug_image/compressed   sensor_msgs/CompressedImage (JPEG, from the overlay)

Parameters (all optional, see declare_parameter below for defaults)
    topic, window_name, image_reliability
Close the window or press q / Esc to quit.
"""

import os
import threading

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

from perception.image_qos import IMAGE_RELIABILITY, image_qos

TOPIC = "/perception/debug_image/compressed"   # default input: detection_overlay's JPEG output
WINDOW_NAME = "HALO perception"
REFRESH_S = 0.005        # main-loop period: well under one frame at 60 Hz
QUIT_KEYS = (ord("q"), 27)   # q and Esc


def decode_jpeg(data):
    """A JPEG byte string as a BGR image, or None when it is empty or not a valid image."""
    if not data:
        return None
    try:
        return cv2.imdecode(np.frombuffer(bytes(data), np.uint8), cv2.IMREAD_COLOR)
    except cv2.error:
        return None


class DebugView(Node):
    """ROS2 node: keeps the newest decoded frame; main() draws it."""

    def __init__(self, **kwargs):
        super().__init__("debug_view", **kwargs)
        self.topic = self.declare_parameter("topic", TOPIC).value
        self.window_name = self.declare_parameter("window_name", WINDOW_NAME).value
        self.image_reliability = self.declare_parameter(
            "image_reliability", IMAGE_RELIABILITY).value
        self._lock = threading.Lock()
        self._latest = None      # newest decoded frame not yet shown
        self.sub = self.create_subscription(
            CompressedImage, self.topic, self.on_image, image_qos(self.image_reliability))
        self.get_logger().info(f"Ready: topic={self.topic}({self.image_reliability})")

    def on_image(self, msg):
        """Decode one JPEG and keep it as the newest frame; a bad frame is skipped."""
        frame = decode_jpeg(msg.data)
        if frame is None:
            self.get_logger().warn("Cannot decode the image, skipping frame",
                                   throttle_duration_sec=5.0)
            return
        with self._lock:
            self._latest = frame

    def take_frame(self):
        """The newest frame the window has not shown yet, or None."""
        with self._lock:
            frame, self._latest = self._latest, None
        return frame


def spin_quietly(node):
    """rclpy.spin for a background thread: a shutdown from the main thread ends it quietly."""
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass


def main(args=None):
    rclpy.init(args=args)
    node = DebugView()
    if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
        node.get_logger().error("No display (DISPLAY is not set): nothing to show")
        node.destroy_node()
        rclpy.shutdown()
        return
    spinner = threading.Thread(target=spin_quietly, args=(node,), daemon=True)
    spinner.start()
    shown = False            # the window exists only after the first frame
    try:
        while rclpy.ok():
            frame = node.take_frame()
            if frame is not None:
                cv2.imshow(node.window_name, frame)
                shown = True
            key = cv2.waitKey(max(1, int(REFRESH_S * 1000))) & 0xFF
            closed = shown and cv2.getWindowProperty(
                node.window_name, cv2.WND_PROP_VISIBLE) < 1
            if key in QUIT_KEYS or closed:
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
