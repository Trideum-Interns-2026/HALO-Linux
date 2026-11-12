"""Pure functions that turn Geolocation / Tracking positions into the web API's JSON.

No ROS calls in here, so it is unit-testable without ROS. Inputs are plain tuples
(latitude_deg, longitude_deg, altitude_m).
"""
import json
import math
from typing import Optional, Tuple

Position = Tuple[float, float, float]


def is_valid_position(position: Position) -> bool:
    """True if latitude/longitude are in range and every value is finite."""
    latitude, longitude, altitude = position
    if not all(math.isfinite(v) for v in position):
        return False
    return -90.0 <= latitude <= 90.0 and -180.0 <= longitude <= 180.0


def is_fresh(stamp_s: float, now_s: float, max_age_s: float) -> bool:
    """True if the stamp is no older than max_age_s (and not from the future by > max_age_s)."""
    return abs(now_s - stamp_s) <= max_age_s


def build_telemetry(drone: Optional[Position], person: Optional[Position]) -> Optional[dict]:
    """Build the API payload. None when there is no valid drone position (the API requires one).

    The person's fields are left out when there is no valid person position.
    """
    if drone is None or not is_valid_position(drone):
        return None
    payload = {"latitude": drone[0], "longitude": drone[1], "altitude_m": drone[2]}
    if person is not None and is_valid_position(person):
        payload["target_latitude"] = person[0]
        payload["target_longitude"] = person[1]
        payload["target_altitude_m"] = person[2]
    return payload


def encode_payload(payload: dict) -> bytes:
    return json.dumps(payload).encode("utf-8")
