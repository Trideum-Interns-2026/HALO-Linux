"""Node tests: real ROS messages, fake HTTP poster. Needs the workspace sourced."""
import threading

import pytest
import rclpy
from geolocation_msgs.msg import GeolocationState
from rclpy.parameter import Parameter
from tracking_msgs.msg import TrackingState

from ui.ui_bridge_node import UiBridgeNode


class FakePoster:
    def __init__(self, error=None):
        self.payloads = []
        self.error = error
        self.event = threading.Event()

    def post(self, payload):
        self.payloads.append(payload)
        self.event.set()
        if self.error:
            raise self.error


@pytest.fixture
def make_node():
    rclpy.init()
    nodes = []

    def make(poster, **params):
        node = UiBridgeNode(poster=poster, parameter_overrides=[
            Parameter(k, value=v) for k, v in params.items()])
        nodes.append(node)
        return node

    yield make
    for node in nodes:
        node.shutdown()
        node.destroy_node()
    rclpy.shutdown()


def now_stamp(node, msg):
    msg.header.stamp = node.get_clock().now().to_msg()
    return msg


def drone_msg(node):
    msg = now_stamp(node, GeolocationState())
    msg.latitude_deg, msg.longitude_deg, msg.altitude_m = 34.73, -86.58, 120.0
    return msg


def person_msg(node):
    msg = now_stamp(node, TrackingState())
    msg.latitude_deg, msg.longitude_deg, msg.altitude_m = 34.7301, -86.5801, 118.5
    return msg


def test_defaults(make_node):
    node = make_node(FakePoster())
    assert node.get_parameter("geolocation_topic").value == "/geolocation/position"
    assert node.get_parameter("tracking_topic").value == "/tracking/position"
    assert node.get_parameter("api_url").value == "http://127.0.0.1:8000/api/telemetry"
    assert node.get_parameter("post_rate_hz").value == 2.0
    assert node.get_parameter("maximum_input_age_s").value == 2.0


def test_posts_drone_and_person(make_node):
    poster = FakePoster()
    node = make_node(poster)
    node._on_drone(drone_msg(node))
    node._on_person(person_msg(node))
    node._on_timer()
    assert poster.event.wait(2.0)
    assert poster.payloads[0] == {
        "latitude": 34.73, "longitude": -86.58, "altitude_m": 120.0,
        "target_latitude": 34.7301, "target_longitude": -86.5801, "target_altitude_m": 118.5}


def test_nothing_posted_without_inputs(make_node):
    poster = FakePoster()
    node = make_node(poster)
    node._on_timer()
    assert not poster.event.wait(0.2)


def test_person_only_posts_nothing(make_node):
    poster = FakePoster()
    node = make_node(poster)
    node._on_person(person_msg(node))
    node._on_timer()
    assert not poster.event.wait(0.2)


def test_stale_person_is_left_out(make_node):
    poster = FakePoster()
    node = make_node(poster, maximum_input_age_s=0.5)
    node._on_drone(drone_msg(node))
    stale = person_msg(node)
    stale.header.stamp.sec -= 10
    node._on_person(stale)
    assert node.build_payload() == {
        "latitude": 34.73, "longitude": -86.58, "altitude_m": 120.0}


def test_stale_drone_posts_nothing(make_node):
    node = make_node(FakePoster(), maximum_input_age_s=0.5)
    old = drone_msg(node)
    old.header.stamp.sec -= 10
    node._on_drone(old)
    assert node.build_payload() is None


def test_post_failure_does_not_crash_and_next_post_still_runs(make_node):
    poster = FakePoster(error=OSError("connection refused"))
    node = make_node(poster)
    node._on_drone(drone_msg(node))
    node._on_timer()
    assert poster.event.wait(2.0)
    for _ in range(100):  # wait for the worker to clear the in-flight flag
        if not node._in_flight:
            break
        threading.Event().wait(0.02)
    assert not node._in_flight
    poster.event.clear()
    node._on_timer()
    assert poster.event.wait(2.0)
    assert len(poster.payloads) == 2
