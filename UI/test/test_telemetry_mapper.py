import math

import pytest

from ui.telemetry_mapper import build_telemetry, encode_payload, is_fresh, is_valid_position

DRONE = (34.73, -86.58, 120.0)
PERSON = (34.7301, -86.5801, 118.5)


@pytest.mark.parametrize("position,expected", [
    ((0.0, 0.0, 0.0), True),
    ((90.0, 180.0, 5.0), True),      # exact limits are valid
    ((-90.0, -180.0, 5.0), True),
    ((90.0001, 0.0, 0.0), False),
    ((0.0, -180.0001, 0.0), False),
    ((math.nan, 0.0, 0.0), False),
    ((0.0, 0.0, math.inf), False),
])
def test_is_valid_position(position, expected):
    assert is_valid_position(position) is expected


@pytest.mark.parametrize("stamp,now,expected", [
    (10.0, 10.5, True),
    (10.0, 12.0, True),    # exactly at the limit is fresh
    (10.0, 12.001, False),
    (12.5, 10.0, False),   # far in the future is not fresh either
])
def test_is_fresh(stamp, now, expected):
    assert is_fresh(stamp, now, 2.0) is expected


def test_drone_and_person():
    assert build_telemetry(DRONE, PERSON) == {
        "latitude": 34.73, "longitude": -86.58, "altitude_m": 120.0,
        "target_latitude": 34.7301, "target_longitude": -86.5801, "target_altitude_m": 118.5}


def test_drone_only_leaves_target_out():
    payload = build_telemetry(DRONE, None)
    assert payload == {"latitude": 34.73, "longitude": -86.58, "altitude_m": 120.0}


def test_nothing_without_a_drone():
    assert build_telemetry(None, PERSON) is None


def test_invalid_drone_gives_nothing_and_invalid_person_is_dropped():
    assert build_telemetry((95.0, 0.0, 0.0), PERSON) is None
    assert "target_latitude" not in build_telemetry(DRONE, (0.0, 200.0, 0.0))


def test_encode_round_trip():
    import json
    payload = build_telemetry(DRONE, PERSON)
    assert json.loads(encode_payload(payload)) == payload
