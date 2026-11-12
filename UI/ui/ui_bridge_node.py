"""UI bridge: latest drone and person positions -> JSON -> ground-station web API (HTTP).

The UI never talks to ROS or the drone directly; this node is the translation layer. It only
reads /geolocation/position and /tracking/position and never commands the vehicle.
"""
import threading
from typing import Optional

import rclpy
from geolocation_msgs.msg import GeolocationState
from rclpy.node import Node
from tracking_msgs.msg import TrackingState

from ui.http_poster import HttpPoster
from ui.telemetry_mapper import build_telemetry, is_fresh

DEFAULT_API_URL = "http://127.0.0.1:8000/api/telemetry"
LOG_THROTTLE_S = 5.0
SHUTDOWN_MARGIN_S = 0.5


class UiBridgeNode(Node):
    def __init__(self, poster=None, **kwargs):
        super().__init__("ui_bridge", **kwargs)
        self._drone_topic = self.declare_parameter(
            "geolocation_topic", "/geolocation/position").value
        self._person_topic = self.declare_parameter(
            "tracking_topic", "/tracking/position").value
        self._api_url = self.declare_parameter("api_url", DEFAULT_API_URL).value
        self._post_rate_hz = float(self.declare_parameter("post_rate_hz", 2.0).value)
        self._max_age_s = float(self.declare_parameter("maximum_input_age_s", 2.0).value)
        self._timeout_s = float(self.declare_parameter("http_timeout_s", 1.0).value)

        self._poster = poster or HttpPoster(self._api_url, self._timeout_s)
        # (received_s, (lat, lon, alt)). Age is counted from when the message arrived on this
        # node's clock, as Movement and Tracking do, so it works with or without use_sim_time.
        self._drone = None
        self._person = None
        self._lock = threading.Lock()
        self._in_flight = False
        self._worker = None

        # Inputs are other services' outputs (reliable, depth 10), not sensors: match that QoS.
        self.create_subscription(GeolocationState, self._drone_topic, self._on_drone, 10)
        self.create_subscription(TrackingState, self._person_topic, self._on_person, 10)
        self._timer = self.create_timer(1.0 / max(self._post_rate_hz, 1e-3), self._on_timer)
        self.get_logger().info(
            f"ui_bridge: {self._drone_topic}, {self._person_topic} -> {self._api_url} "
            f"at {self._post_rate_hz} Hz, max input age {self._max_age_s} s")

    def _on_drone(self, msg: GeolocationState) -> None:
        self._drone = (self._now_s(), (msg.latitude_deg, msg.longitude_deg, msg.altitude_m))

    def _on_person(self, msg: TrackingState) -> None:
        self._person = (self._now_s(), (msg.latitude_deg, msg.longitude_deg, msg.altitude_m))

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _fresh_position(self, entry, now_s: float):
        if entry is None or not is_fresh(entry[0], now_s, self._max_age_s):
            return None
        return entry[1]

    def build_payload(self) -> Optional[dict]:
        now_s = self._now_s()
        return build_telemetry(self._fresh_position(self._drone, now_s),
                               self._fresh_position(self._person, now_s))

    def _on_timer(self) -> None:
        payload = self.build_payload()
        if payload is None:
            return  # nothing fresh to show: publish nothing
        with self._lock:
            if self._in_flight:
                return  # previous POST still running; never block or queue up
            self._in_flight = True
        self._worker = threading.Thread(target=self._send, args=(payload,), daemon=True)
        self._worker.start()

    def _send(self, payload: dict) -> None:
        try:
            self._poster.post(payload)
        except Exception as error:  # web API down must never crash the bridge
            self.get_logger().warning(
                f"could not post to {self._api_url}: {error}",
                throttle_duration_sec=LOG_THROTTLE_S)
        finally:
            with self._lock:
                self._in_flight = False

    def shutdown(self) -> None:
        """Stop posting and wait for a POST in flight, so it cannot log on a destroyed node."""
        self._timer.cancel()
        if self._worker is not None:
            self._worker.join(timeout=self._timeout_s + SHUTDOWN_MARGIN_S)


def main(args=None):
    rclpy.init(args=args)
    node = UiBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
