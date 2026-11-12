"""Tests for the detection overlay: the drawing and timing functions, then the node.

Needs the workspace sourced (source /opt/ros2_ws/install/setup.bash), but no sim and no Triton.
"""
import math
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest
import rclpy
from cv_bridge import CvBridge
from perception_msgs.msg import Detection
from sensor_msgs.msg import Image

from perception.detection_overlay import (
    BOX_COLOR_BGR,
    COORD_LIMIT_PX,
    DEFAULT_MAX_AGE_S,
    DEFAULT_SCALE,
    MAX_SCALE,
    MIN_SCALE,
    DetectionOverlay,
    draw_detection,
    is_fresh,
    sanitize_scale,
    scale_box,
    stamp_to_s,
)

# ---------- logic: scale, stamps, freshness ----------


@pytest.mark.parametrize("value, expected", [
    (0.5, 0.5),
    (1.0, 1.0),
    (MIN_SCALE, MIN_SCALE),
    (2.0, MAX_SCALE),            # above the ceiling
    (0.0, MIN_SCALE),            # below the floor
    (-1.0, MIN_SCALE),
    (float("nan"), DEFAULT_SCALE),
    (float("inf"), DEFAULT_SCALE),
    ("not a number", DEFAULT_SCALE),
    (None, DEFAULT_SCALE),
])
def test_sanitize_scale(value, expected):
    assert sanitize_scale(value) == pytest.approx(expected)


def test_stamp_to_seconds():
    stamp = SimpleNamespace(sec=2, nanosec=500_000_000)
    assert stamp_to_s(stamp) == pytest.approx(2.5)


@pytest.mark.parametrize("detection_s, image_s, expected", [
    (10.0, 10.0, True),
    (10.0, 10.5, True),          # exactly at the limit is still fresh
    (10.5, 10.0, True),          # either order
    (10.0, 10.5001, False),      # just over
    (9.0, 10.0, False),
    (float("nan"), 10.0, False),
    (10.0, float("inf"), False),
])
def test_is_fresh(detection_s, image_s, expected):
    assert is_fresh(detection_s, image_s, DEFAULT_MAX_AGE_S) is expected


def test_is_fresh_rejects_a_non_finite_limit():
    assert is_fresh(10.0, 10.0, float("nan")) is False


@pytest.mark.parametrize("box, scale, expected", [
    ((100, 200, 300, 400), 0.5, (50, 100, 150, 200)),
    ((100, 200, 300, 400), 1.0, (100, 200, 300, 400)),
    ((101, 201, 301, 401), 0.5, (50, 100, 150, 200)),    # rounds to the nearest pixel
    ((-50.0, -20.0, 10.0, 10.0), 0.5, (-25, -10, 5, 5)),  # outside the frame is kept as is
    ((0, 0, 1e12, -1e12), 1.0, (0, 0, COORD_LIMIT_PX, -COORD_LIMIT_PX)),   # absurd values clamp
])
def test_scale_box(box, scale, expected):
    assert scale_box(box, scale) == expected


# ---------- logic: drawing ----------


def blank(h=200, w=300):
    return np.zeros((h, w, 3), dtype=np.uint8)


def test_draw_detection_marks_the_box_edge_and_leaves_the_rest_alone():
    frame = blank()
    out = draw_detection(frame, (50, 60, 150, 160), 0.95)
    assert out is frame                                         # drawn in place
    assert tuple(frame[110, 50]) == BOX_COLOR_BGR               # left edge, mid height
    assert tuple(frame[160, 100]) == BOX_COLOR_BGR              # bottom edge, mid width
    assert not frame[100:150, 70:130].any()                     # inside the box is untouched
    assert not frame[0:20, 200:].any()                          # far from the box is untouched
    assert frame[40:60, 50:100].any()                           # the label sits above the box


def test_draw_detection_label_moves_inside_when_the_box_touches_the_top():
    frame = blank()
    draw_detection(frame, (50, 0, 150, 100), 0.5)               # must not raise or draw off-frame
    assert frame[2:20, 52:100].any()                            # label inside the top edge


@pytest.mark.parametrize("box", [
    (-100, -100, -10, -10),                  # wholly outside
    (250, 150, 500, 400),                    # partly outside
    (60, 60, 50, 50),                        # inverted
    (-COORD_LIMIT_PX, -COORD_LIMIT_PX, COORD_LIMIT_PX, COORD_LIMIT_PX),
    (10, 10, 10, 10),                        # zero size
])
def test_draw_detection_never_raises_for_odd_boxes(box):
    draw_detection(blank(), box, 0.9)


# ---------- node ----------


@pytest.fixture
def node():
    """A real overlay node whose publisher is swapped for a fake that records publish() calls."""
    rclpy.init()
    n = DetectionOverlay()
    n.pub = MagicMock()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def camera_image(sec=10, width=640, height=360):
    msg = CvBridge().cv2_to_imgmsg(np.zeros((height, width, 3), dtype=np.uint8), encoding="bgr8")
    msg.header.stamp.sec = sec
    msg.header.stamp.nanosec = 456
    msg.header.frame_id = "camera_link"
    return msg


def detection(sec=10, confidence=0.9, box=(100.0, 100.0, 300.0, 300.0)):
    msg = Detection()
    msg.header.stamp.sec = sec
    msg.confidence = confidence
    msg.x_min, msg.y_min, msg.x_max, msg.y_max = box
    return msg


def published_frame(node):
    out = node.pub.publish.call_args[0][0]
    return out, CvBridge().imgmsg_to_cv2(out, desired_encoding="bgr8")


def test_parameters_have_documented_defaults(node):
    assert node.get_parameter("image_topic").value == "/camera/front/image_raw"
    assert node.get_parameter("detection_topic").value == "/perception/detection"
    assert node.get_parameter("output_topic").value == "/perception/debug_image"
    assert node.get_parameter("image_reliability").value == "reliable"
    assert node.get_parameter("scale").value == pytest.approx(0.5)
    assert node.get_parameter("max_age_s").value == pytest.approx(0.5)


def test_draws_a_fresh_box_scaled_and_copies_the_header(node):
    node.on_detection(detection(sec=10))
    node.on_image(camera_image(sec=10))
    out, frame = published_frame(node)
    assert frame.shape == (180, 320, 3)                          # scale 0.5 of 640x360
    assert out.header.stamp.sec == 10 and out.header.stamp.nanosec == 456
    assert out.header.frame_id == "camera_link"
    assert out.encoding == "bgr8"
    assert tuple(frame[100, 50]) == BOX_COLOR_BGR                # box x_min 100 -> 50, mid height


def test_scale_one_keeps_the_size_and_the_box_position(node):
    node.scale = 1.0
    node.on_detection(detection(sec=10))
    node.on_image(camera_image(sec=10))
    _, frame = published_frame(node)
    assert frame.shape == (360, 640, 3)
    assert tuple(frame[200, 100]) == BOX_COLOR_BGR


def test_stale_detection_is_not_drawn_but_the_image_is_still_published(node):
    node.on_detection(detection(sec=20))                         # 10 s away from the image
    node.on_image(camera_image(sec=10))
    node.pub.publish.assert_called_once()
    _, frame = published_frame(node)
    assert not frame.any()


def test_no_detection_yet_publishes_the_plain_image(node):
    node.on_image(camera_image())
    node.pub.publish.assert_called_once()
    _, frame = published_frame(node)
    assert not frame.any()


def test_newest_detection_wins(node):
    node.on_detection(detection(sec=10, box=(100.0, 100.0, 300.0, 300.0)))
    node.on_detection(detection(sec=10, box=(400.0, 100.0, 600.0, 300.0)))
    node.on_image(camera_image(sec=10))
    _, frame = published_frame(node)
    assert tuple(frame[100, 200]) == BOX_COLOR_BGR               # second box x_min 400 -> 200
    assert not frame[100, 50].any()                              # the first box is gone


def test_non_finite_detection_is_ignored(node):
    node.on_detection(detection(sec=10))
    node.on_detection(detection(sec=10, box=(math.nan, 0.0, 10.0, 10.0)))
    node.on_image(camera_image(sec=10))
    _, frame = published_frame(node)
    assert tuple(frame[100, 50]) == BOX_COLOR_BGR                # still the earlier good box


def test_unreadable_image_is_skipped_without_crashing(node):
    bad = Image()
    bad.encoding = "not_an_encoding"
    bad.height = 4
    bad.width = 4
    bad.step = 12
    bad.data = [0] * 48
    node.on_image(bad)                                           # must not raise
    node.pub.publish.assert_not_called()
