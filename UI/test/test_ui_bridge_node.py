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
    node._on_person(person_msg(node))
    node._person = (node._person[0] - 10.0, node._person[1])   # received 10 s ago
    assert node.build_payload() == {
        "latitude": 34.73, "longitude": -86.58, "altitude_m": 120.0}


def test_stale_drone_posts_nothing(make_node):
    node = make_node(FakePoster(), maximum_input_age_s=0.5)
    node._on_drone(drone_msg(node))
    node._drone = (node._drone[0] - 10.0, node._drone[1])   # received 10 s ago
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


def test_non_http_api_url_is_rejected_at_startup():
    rclpy.init()
    try:
        with pytest.raises(ValueError):
            UiBridgeNode(parameter_overrides=[Parameter("api_url", value="file:///etc/passwd")])
    finally:
        rclpy.shutdown()


def test_shutdown_waits_for_a_post_in_flight(make_node):
    release = threading.Event()

    class SlowPoster(FakePoster):
        def post(self, payload):
            super().post(payload)
            release.wait(2.0)

    poster = SlowPoster()
    node = make_node(poster)
    node._on_drone(drone_msg(node))
    node._on_timer()
    assert poster.event.wait(2.0)
    assert node._in_flight
    threading.Timer(0.2, release.set).start()
    node.shutdown()
    assert not node._in_flight


def test_second_tick_is_skipped_while_a_post_is_in_flight(make_node):
    release = threading.Event()

    class SlowPoster(FakePoster):
        def post(self, payload):
            super().post(payload)
            release.wait(2.0)

    poster = SlowPoster()
    node = make_node(poster)
    node._on_drone(drone_msg(node))
    node._on_timer()
    assert poster.event.wait(2.0)
    node._on_timer()
    node._on_timer()
    release.set()
    node.shutdown()
    assert len(poster.payloads) == 1


def test_sim_time_stamps_are_accepted_without_use_sim_time(make_node):
    # In the sim, header stamps are seconds since sim start (e.g. 27 s) while the node clock is
    # wall time. Freshness counts from arrival, so the bridge still posts.
    poster = FakePoster()
    node = make_node(poster)
    drone = drone_msg(node)
    drone.header.stamp.sec, drone.header.stamp.nanosec = 27, 0
    person = person_msg(node)
    person.header.stamp.sec, person.header.stamp.nanosec = 27, 0
    node._on_drone(drone)
    node._on_person(person)
    payload = node.build_payload()
    assert payload["latitude"] == 34.73
    assert payload["target_latitude"] == 34.7301
