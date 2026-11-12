"""Tests for the debug viewer's decoding and newest-frame hand-off; no display is opened.

Needs the workspace sourced (source /opt/ros2_ws/install/setup.bash).
"""
import cv2
import numpy as np
import pytest
import rclpy
from sensor_msgs.msg import CompressedImage

from perception.debug_view import DebugView, decode_jpeg


def jpeg(width=64, height=48, value=200):
    ok, buf = cv2.imencode(".jpg", np.full((height, width, 3), value, np.uint8))
    assert ok
    return buf.tobytes()


def packed(data):
    msg = CompressedImage()
    msg.format = "jpeg"
    msg.data = data
    return msg


@pytest.fixture
def node():
    rclpy.init()
    n = DebugView()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def test_decode_jpeg_round_trip_shape_and_colour():
    frame = decode_jpeg(jpeg(64, 48, 200))
    assert frame.shape == (48, 64, 3)
    assert abs(int(frame.mean()) - 200) <= 3          # JPEG is lossy, but close


@pytest.mark.parametrize("data", [b"", None, b"not a jpeg", bytes(range(50))])
def test_decode_jpeg_bad_input_is_none(data):
    assert decode_jpeg(data) is None


def test_parameters_have_documented_defaults(node):
    assert node.get_parameter("topic").value == "/perception/debug_image/compressed"
    assert node.get_parameter("window_name").value == "HALO perception"
    assert node.get_parameter("image_reliability").value == "reliable"


def test_keeps_only_the_newest_frame_and_clears_it_when_taken(node):
    assert node.take_frame() is None
    node.on_image(packed(jpeg(value=50)))
    node.on_image(packed(jpeg(value=250)))             # replaces the older one
    frame = node.take_frame()
    assert abs(int(frame.mean()) - 250) <= 3
    assert node.take_frame() is None


def test_bad_frame_is_skipped_without_crashing_or_losing_the_good_one(node):
    node.on_image(packed(jpeg(value=120)))
    node.on_image(packed(b"garbage"))                  # must not raise or overwrite
    assert abs(int(node.take_frame().mean()) - 120) <= 3
