"""Node-level tests: real ROS messages in and out, with a fake Triton.

Needs the workspace sourced (source /opt/ros2_ws/install/setup.bash),
but no sim and no Triton server.
"""
from unittest.mock import MagicMock

import threading
import time

import numpy as np
import pytest
import rclpy
from cv_bridge import CvBridge
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, ReliabilityPolicy
from tritonclient.utils import InferenceServerException

from perception.yolo_person_detector import YoloPersonDetector
from perception_msgs.msg import Detection


# The pass-through setup most tests use: publish on the first frame, box exactly as decoded.
PASS_THROUGH = {"confirm_frames": 1, "smooth_box": False, "async_inference": False}


@pytest.fixture
def make_node(monkeypatch):
    """Factory for real detector nodes whose publisher is a fake; parameters via overrides."""
    # The Triton client only connects on the first infer(), so any URL works here
    monkeypatch.setenv("HALO_TRITON_URL", "localhost:8001")
    rclpy.init()
    nodes = []

    def make(**params):
        n = YoloPersonDetector(parameter_overrides=[Parameter(k, value=v)
                                                    for k, v in params.items()])
        n.pub = MagicMock()          # records publish() calls instead of sending them
        nodes.append(n)
        return n

    yield make
    for n in nodes:
        n.shutdown()
        n.destroy_node()
    rclpy.shutdown()


@pytest.fixture
def node(make_node):
    """A detector with the gate and smoother out of the way (they have their own tests)."""
    return make_node(**PASS_THROUGH)


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


def test_parameters_have_documented_defaults(make_node):
    """Every setting is a ROS parameter with the default the README documents."""
    node = make_node()
    assert node.get_parameter("triton_url").value == "localhost:8001"   # from HALO_TRITON_URL
    assert node.get_parameter("model_name").value == "yolo_person"
    assert node.get_parameter("min_confidence").value == pytest.approx(0.7)
    assert node.get_parameter("keep_confidence").value == pytest.approx(0.5)
    assert node.get_parameter("confirm_frames").value == 3
    assert node.get_parameter("max_missed_frames").value == 5
    assert node.get_parameter("smooth_box").value is True
    assert node.get_parameter("async_inference").value is True
    assert node.get_parameter("timing_report_s").value == pytest.approx(0.0)
    assert node.get_parameter("triton_timeout_s").value == pytest.approx(0.1)
    assert node.get_parameter("image_topic").value == "/camera/front/image_raw"
    assert node.get_parameter("detection_topic").value == "/perception/detection"
    assert node.get_parameter("image_reliability").value == "reliable"


@pytest.mark.parametrize("value, expected", [
    ("reliable", ReliabilityPolicy.RELIABLE),
    ("best_effort", ReliabilityPolicy.BEST_EFFORT),
    ("BEST_EFFORT", ReliabilityPolicy.BEST_EFFORT),   # case does not matter
    ("nonsense", ReliabilityPolicy.RELIABLE),         # unknown falls back to the sim default
    (None, ReliabilityPolicy.RELIABLE),
])
def test_image_qos_reliability_and_newest_frame_only(value, expected):
    """The camera QoS follows image_reliability and always keeps only the newest frame."""
    qos = YoloPersonDetector.image_qos(value)
    assert qos.reliability == expected
    assert qos.history == HistoryPolicy.KEEP_LAST
    assert qos.depth == 1


def test_min_confidence_parameter_is_used(make_node):
    """Raising min_confidence above the score suppresses the detection."""
    node = make_node(min_confidence=0.95, **PASS_THROUGH)
    node._infer = lambda blob: fake_output(0.9)
    node.on_image(camera_image())
    node.pub.publish.assert_not_called()


# ---------- gate, smoothing, worker, timing (new behaviour) ----------


def raw_with(*people):
    """Triton-shaped output: people = (index, cx, cy, w, h, score) in 640-letterbox pixels."""
    raw = np.zeros((1, 84, 8400), dtype=np.float32)
    for i, cx, cy, w, h, score in people:
        raw[0, 0:4, i] = [cx, cy, w, h]
        raw[0, 4, i] = score
    return raw


PERSON = (10, 320, 290, 40, 100, 0.9)        # tall box, aspect 2.5
TREE_WIDE = (20, 100, 200, 120, 80, 0.95)    # wider than tall: not person-shaped
TREE_60 = (30, 500, 300, 50, 130, 0.6)       # person-shaped but only 0.6


def test_a_tree_scoring_0_6_is_never_published_with_defaults(make_node):
    node = make_node(async_inference=False)
    node._infer = lambda blob: raw_with(TREE_60)
    for _ in range(10):
        node.on_image(camera_image())
    node.pub.publish.assert_not_called()


def test_wide_box_is_rejected_even_at_high_confidence(make_node):
    node = make_node(async_inference=False, confirm_frames=1)
    node._infer = lambda blob: raw_with(TREE_WIDE)
    node.on_image(camera_image())
    node.pub.publish.assert_not_called()


def test_wide_loud_box_does_not_hide_the_real_person(make_node):
    node = make_node(async_inference=False, confirm_frames=1, smooth_box=False)
    node._infer = lambda blob: raw_with(TREE_WIDE, PERSON)
    node.on_image(camera_image())
    out = node.pub.publish.call_args.args[0]
    assert out.confidence == pytest.approx(0.9)


def test_default_publishes_only_after_three_confirming_frames(make_node):
    node = make_node(async_inference=False)
    node._infer = lambda blob: raw_with(PERSON)
    node.on_image(camera_image())
    node.on_image(camera_image())
    node.pub.publish.assert_not_called()
    node.on_image(camera_image())
    node.pub.publish.assert_called_once()


def test_track_survives_a_low_confidence_frame_then_is_dropped(make_node):
    node = make_node(async_inference=False, confirm_frames=1, max_missed_frames=1,
                     smooth_box=False)
    low = (10, 320, 290, 40, 100, 0.55)       # below min_confidence, above keep_confidence
    node._infer = lambda blob: raw_with(PERSON)
    node.on_image(camera_image())
    node._infer = lambda blob: raw_with(low)
    node.on_image(camera_image())
    assert node.pub.publish.call_count == 2             # kept alive by keep_confidence
    node._infer = lambda blob: raw_with()
    node.on_image(camera_image())                       # miss 1
    node.on_image(camera_image())                       # miss 2 > max_missed 1 -> dropped
    node._infer = lambda blob: raw_with(low)
    node.on_image(camera_image())
    assert node.pub.publish.call_count == 2             # 0.55 cannot start a new track


def test_smoothing_blends_the_second_box(make_node):
    node = make_node(async_inference=False, confirm_frames=1, smoothing_alpha=0.5,
                     smoothing_beta=0.0)
    node._infer = lambda blob: raw_with(PERSON)
    first = camera_image()
    node.on_image(first)
    moved = (10, 332, 290, 40, 100, 0.9)                # 12 letterbox px -> 36 frame px right
    node._infer = lambda blob: raw_with(moved)
    second = camera_image()
    second.header.stamp.nanosec = 456 + 33_000_000
    node.on_image(second)
    out = node.pub.publish.call_args.args[0]
    assert 900 < out.x_min < 936                         # between old (900) and measured (936)
    assert out.x_min == pytest.approx(918, abs=0.5)


def test_published_box_is_clipped_to_the_frame(make_node):
    node = make_node(async_inference=False, confirm_frames=1)
    node._infer = lambda blob: raw_with((10, 630, 290, 60, 100, 0.9))   # sticks out on the right
    node.on_image(camera_image())
    assert node.pub.publish.call_args.args[0].x_max <= 1920.0


def test_worker_processes_only_the_newest_waiting_frame(make_node):
    node = make_node(async_inference=False, confirm_frames=1)
    seen = []
    node.on_image = lambda msg: seen.append(msg.header.stamp.sec)
    older, newer = camera_image(), camera_image()
    older.header.stamp.sec, newer.header.stamp.sec = 1, 2
    node.on_image_async(older)
    node.on_image_async(newer)                           # replaces the older one
    node._worker_thread = threading.Thread(target=node._worker, daemon=True)
    node._worker_thread.start()
    deadline = time.monotonic() + 2.0
    while not seen and time.monotonic() < deadline:
        time.sleep(0.01)
    assert seen == [2]


def test_worker_survives_a_failing_frame(make_node):
    node = make_node(async_inference=True)
    done = threading.Event()
    calls = []

    def flaky(msg):
        calls.append(msg.header.stamp.sec)
        if len(calls) == 1:
            raise RuntimeError("bad frame")
        done.set()

    node.on_image = flaky
    first, second = camera_image(), camera_image()
    first.header.stamp.sec, second.header.stamp.sec = 1, 2
    node.on_image_async(first)
    deadline = time.monotonic() + 2.0
    while not calls and time.monotonic() < deadline:
        time.sleep(0.01)
    node.on_image_async(second)
    assert done.wait(2.0)


def test_shutdown_stops_the_worker(make_node):
    node = make_node(async_inference=True)
    assert node._worker_thread.is_alive()
    node.shutdown()
    assert not node._worker_thread.is_alive()


def test_timing_summary_is_logged_when_enabled(make_node):
    node = make_node(**PASS_THROUGH, timing_report_s=0.001)
    node._infer = lambda blob: raw_with(PERSON)
    logged = []
    node.get_logger().info = lambda m, **k: logged.append(m)
    node.on_image(camera_image())
    time.sleep(0.01)
    node.on_image(camera_image())
    assert any("timing:" in m and "triton" in m for m in logged)


def test_no_timing_work_when_disabled(node):
    node._infer = lambda blob: raw_with(PERSON)
    node.on_image(camera_image())
    assert node.timer.frames == 0
