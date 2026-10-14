"""Minimal live viewer for a detector's debug image, used by
run_perception_demo.sh in place of rqt_image_view.

rqt_image_view subscribes BEST_EFFORT, and on this setup (Fast DDS, ~1.5MB
debug frames) best-effort drops most frames: measured in the sim it showed
~0.6 Hz with gaps of up to 5s while the detector was publishing far faster.
This subscribes RELIABLE with depth 1 (matching the detectors' publishers),
so every frame gets through but never queues up, and always draws the newest
one. It logs how many frames it received and actually drew, so a laggy
viewer can be told apart from a slow topic.

Usage: ros2 run perception debug_image_viewer <image_topic>
Press q or Esc in the window (or Ctrl+C) to quit.
"""

import sys
import threading
import time

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class DebugImageViewer(Node):
    def __init__(self, topic):
        super().__init__("debug_image_viewer")
        self.topic = topic
        self.bridge = CvBridge()
        self._lock = threading.Lock()
        self._latest = None  # newest decoded frame not yet drawn
        self._received = 0
        self._shown = 0
        self._since = time.monotonic()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Image, topic, self.on_image, qos)
        self.create_timer(5.0, self._log_stats)
        self.get_logger().info(f"Viewing '{topic}'")

    def on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().warn(f"cv_bridge conversion failed: {exc}")
            return
        with self._lock:
            self._latest = frame
            self._received += 1

    def take_latest(self):
        with self._lock:
            frame, self._latest = self._latest, None
            if frame is not None:
                self._shown += 1
            return frame

    def _log_stats(self):
        with self._lock:
            received, shown = self._received, self._shown
            self._received = self._shown = 0
        elapsed = time.monotonic() - self._since
        self._since = time.monotonic()
        self.get_logger().info(
            f"received {received / elapsed:.1f} Hz, displayed {shown / elapsed:.1f} Hz"
        )


def _spin(node):
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass  # Ctrl+C, or main() shutting down after the window closed


def main(args=None):
    rclpy.init(args=args)
    argv = rclpy.utilities.remove_ros_args(sys.argv)
    topic = argv[1] if len(argv) > 1 else "/yolo_person_detector/person/debug_image"
    node = DebugImageViewer(topic)

    # ROS callbacks on a background thread; the window has to live on the
    # main thread for Qt.
    spinner = threading.Thread(target=_spin, args=(node,), daemon=True)
    spinner.start()

    window = topic
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    drawn = False
    try:
        while rclpy.ok():
            frame = node.take_latest()
            if frame is not None:
                cv2.imshow(window, frame)
                drawn = True
            key = cv2.waitKey(10) & 0xFF
            if key in (ord("q"), 27):
                break
            if drawn and cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                break  # window closed
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.try_shutdown()
        spinner.join(timeout=2.0)
        node.destroy_node()


if __name__ == "__main__":
    main()
