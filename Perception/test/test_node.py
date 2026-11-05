"""Node-level tests: real ROS messages in and out, with a fake Triton.

Needs the workspace sourced (source /opt/ros2_ws/install/setup.bash),
but no sim and no Triton server.
"""
from unittest.mock import MagicMock

import numpy as np
import pytest
import rclpy
from cv_bridge import CvBridge
from tritonclient.utils import InferenceServerException

from perception.yolo_person_detector import YoloPersonDetector
from perception_msgs.msg import Detection


@pytest.fixture
def node(monkeypatch):
    """A real detector node whose Triton call and publisher are swapped for fakes."""
    # The Triton client only connects on the first infer(), so any URL works here
    monkeypatch.setenv("HALO_TRITON_URL", "localhost:8001")
    rclpy.init()
    n = YoloPersonDetector()
    n.pub = MagicMock()          # records publish() calls instead of sending them
    yield n
    n.destroy_node()
    rclpy.shutdown()


def camera_image():
    """A 1920x1080 camera Image with a recognizable timestamp."""
    msg = CvBridge().cv2_to_imgmsg(np.zeros((1080, 1920, 3), dtype=np.uint8), encoding="bgr8")
    msg.header.stamp.sec = 123
    msg.header.stamp.nanosec = 456
    msg.header.frame_id = "camera_link"
    return msg


def fake_output(score):
    """Triton-shaped output with one person at frame box (900,300)-(1020,600)."""
    raw = np.zeros((1, 84, 8400), dtype=np.float32)
    raw[0, 0:4, 123] = [320, 290, 40, 100]
    raw[0, 4, 123] = score
    return raw


def test_publishes_detection_for_person(node):
    """Person in view -> exactly one Detection with the camera's header and frame-pixel box."""
    node._infer = lambda blob: fake_output(0.9)
    node.on_image(camera_image())

    node.pub.publish.assert_called_once()
    out = node.pub.publish.call_args.args[0]
    assert isinstance(out, Detection)
    assert (out.header.stamp.sec, out.header.stamp.nanosec) == (123, 456)
    assert out.header.frame_id == "camera_link"
    assert out.confidence == pytest.approx(0.9)
    assert (out.x_min, out.y_min, out.x_max, out.y_max) == pytest.approx((900, 300, 1020, 600))


def test_sends_model_ready_blob_to_triton(node):
    """What reaches _infer is the (1, 3, 640, 640) float32 blob Triton expects."""
    seen = {}

    def fake_infer(blob):
        seen["blob"] = blob
        return fake_output(0.0)

    node._infer = fake_infer
    node.on_image(camera_image())
    assert seen["blob"].shape == (1, 3, 640, 640)
    assert seen["blob"].dtype == np.float32


def test_no_person_publishes_nothing(node):
    """Empty scene -> no message at all (we only publish on detection)."""
    node._infer = lambda blob: fake_output(0.0)
    node.on_image(camera_image())
    node.pub.publish.assert_not_called()


def test_triton_failure_skips_frame(node):
    """Triton timeout/down -> frame skipped, node keeps running, nothing published."""
    def broken_infer(blob):
        raise InferenceServerException("Deadline Exceeded")

    node._infer = broken_infer
    node.on_image(camera_image())          # must not raise
    node.pub.publish.assert_not_called()
