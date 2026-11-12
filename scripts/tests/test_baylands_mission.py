"""Logic tests for scripts/baylands_mission.py. No sim, ROS or MAVLink needed: only the pure
functions and the refusals that happen before anything is started."""
import importlib.machinery
import importlib.util
import math
import os

import pytest

SCRIPTS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def load():
    loader = importlib.machinery.SourceFileLoader(
        "baylands_mission", os.path.join(SCRIPTS, "baylands_mission.py"))
    spec = importlib.util.spec_from_loader("baylands_mission", loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


bm = load()


# ------------------------------------------------------------------ geometry

def test_plan_from_the_default_launch_point():
    g = bm.plan(bm.LAUNCH_XY, bm.PERSON_XY)
    assert g["distance_m"] == pytest.approx(103.0, abs=0.1)
    assert g["person_ned"] == pytest.approx((15.48, 101.87), abs=1e-6)
    assert math.degrees(g["yaw_rad"]) == pytest.approx(8.64, abs=0.01)
    assert g["model_pose"].startswith("-101.870,-15.480,0,0,0,")


@pytest.mark.parametrize("drone,person", [
    ((-100.0, 0.0), (0.0, 0.0)), ((50.0, 50.0), (0.0, 0.0)), ((0.0, -80.0), (0.0, 0.0)),
    ((30.0, -40.0), (-20.0, 60.0)), ((-101.87, -15.48), (86.0, 53.0)),
])
def test_plan_faces_each_other(drone, person):
    g = bm.plan(drone, person)
    yaw, d = g["yaw_rad"], math.dist(drone, person)
    assert -math.pi <= yaw <= math.pi
    # the drone's nose points at the person...
    assert (math.cos(yaw), math.sin(yaw)) == pytest.approx(
        ((person[0] - drone[0]) / d, (person[1] - drone[1]) / d), abs=1e-9)
    # ...and the person model (looks along -x at yaw 0) looks back at the drone
    assert (-math.cos(yaw), -math.sin(yaw)) == pytest.approx(
        ((drone[0] - person[0]) / d, (drone[1] - person[1]) / d), abs=1e-9)
    assert g["person_ned"] == pytest.approx((person[1] - drone[1], person[0] - drone[0]))


@pytest.mark.parametrize("person,ok", [
    ((39.9, 0.0), False), ((40.0, 0.0), True), ((300.0, 0.0), True), ((300.1, 0.0), False),
    ((0.0, 0.0), False),
])
def test_plan_distance_limits(person, ok):
    if ok:
        assert bm.plan((0.0, 0.0), person)["distance_m"] == pytest.approx(math.hypot(*person))
    else:
        with pytest.raises(ValueError):
            bm.plan((0.0, 0.0), person)


@pytest.mark.parametrize("text,xy", [("1,2", (1.0, 2.0)), ("-101.87,-15.48", (-101.87, -15.48)),
                                     (" 3.5 , -4 ", (3.5, -4.0))])
def test_parse_xy(text, xy):
    assert bm.parse_xy(text) == xy


@pytest.mark.parametrize("text", ["", "1", "1,2,3", "a,b", "nan,1", "1,inf", "1;2"])
def test_parse_xy_rejects_garbage(text):
    with pytest.raises(ValueError):
        bm.parse_xy(text)


def test_leg_offset_is_capped_and_points_at_the_person():
    dn, de, remaining = bm.leg_offset((0.0, 0.0), (60.0, 80.0), 30.0)
    assert remaining == pytest.approx(100.0)
    assert math.hypot(dn, de) == pytest.approx(30.0)
    assert (dn, de) == pytest.approx((18.0, 24.0))


@pytest.mark.parametrize("pos,person", [((0, 0), (3, 4)), ((10, 10), (10, 10)), ((5, 5), (5, 25))])
def test_leg_offset_short_distances_go_all_the_way(pos, person):
    dn, de, remaining = bm.leg_offset(pos, person)
    assert (dn, de) == (person[0] - pos[0], person[1] - pos[1])
    assert remaining == pytest.approx(math.hypot(dn, de))


@pytest.mark.parametrize("lat", [0.0, 37.4, 47.397971, -33.9])
def test_offset_latlon_round_trip(lat):
    lat2, lon2 = bm.offset_latlon(lat, 8.5, 30.0, 40.0)
    north = math.radians(lat2 - lat) * bm.EARTH_RADIUS_M
    east = math.radians(lon2 - 8.5) * bm.EARTH_RADIUS_M * math.cos(math.radians(lat))
    assert north == pytest.approx(30.0, abs=1e-6)
    assert east == pytest.approx(40.0, abs=1e-6)


# ------------------------------------------------------------------ limits are enforced

@pytest.mark.parametrize("n,e,v,h", [
    (0, 0, 1, 6.0), (30, 0, 5, 6.1), (0, -30, 5, 1.2), (18, 24, 0.1, 10.0),
])
def test_check_leg_accepts_in_limit_legs(n, e, v, h):
    bm.check_leg(n, e, v, h)


@pytest.mark.parametrize("n,e,v,h", [
    (30.01, 0, 5, 6.0), (0, 30.01, 5, 6.0), (22, 22, 5, 6.0),           # longer than MAX_LEG_M
    (10, 10, 0, 6.0), (10, 10, -1, 6.0), (10, 10, 5.01, 6.0),           # speed outside (0, MAX]
    (10, 10, 5, 1.19), (10, 10, 5, 10.01),                              # height outside the band
    (float("nan"), 0, 5, 6.0), (0, float("inf"), 5, 6.0), (1, 1, 5, float("nan")),
])
def test_check_leg_refuses_out_of_limit_legs(n, e, v, h):
    with pytest.raises(ValueError):
        bm.check_leg(n, e, v, h)


def test_the_limits_are_the_ones_agents_md_allows():
    assert bm.CRUISE_ALT_M == pytest.approx(6.096, abs=1e-3)           # 20 ft, as the user asked
    assert bm.CRUISE_ALT_M <= 10.0 and bm.MAX_ALT_M <= 10.0            # takeoff cap
    assert bm.MIN_ALT_M <= bm.FOLLOW_ALT_M <= bm.CRUISE_ALT_M <= bm.MAX_ALT_M
    assert bm.MAX_LEG_M <= 60.0 and bm.MAX_SPEED_MPS <= 8.0            # the sim helper's limits
    assert bm.GIVE_UP_M > bm.FOLLOW_DISTANCE_M
    assert bm.MAX_MISSION_M <= 300.0


def test_only_the_local_mavlink_bridge_is_used():
    assert bm.ENDPOINT == ("127.0.0.1", 5760)
    text = open(os.path.join(SCRIPTS, "baylands_mission.py")).read()
    assert "/dev/tty" not in text and "serial" not in text.lower().replace("serial ", "")


# ------------------------------------------------------------------ small decisions

@pytest.mark.parametrize("measured,true,ok", [
    (100.0, 100.0, True), (75.0, 100.0, True), (74.9, 100.0, False),    # 25 % of 100 m
    (10.0, 12.9, True), (10.0, 14.1, False),                            # 3 m floor at short range
])
def test_range_ok(measured, true, ok):
    assert bm.range_ok(measured, true) is ok


@pytest.mark.parametrize("d,ok", [(6.0, True), (3.5, True), (8.5, True), (3.49, False),
                                  (8.51, False), (25.0, False)])
def test_in_follow_band(d, ok):
    assert bm.in_follow_band(d) is ok


@pytest.mark.parametrize("times,now,seen", [
    ([], 10.0, False), ([9.0], 10.0, False), ([9.0, 9.5], 10.0, True),
    ([7.0, 9.5], 10.0, False),                       # the first one is too old
    ([8.0, 9.0], 10.0, True),                        # exactly at the window edge counts
    ([7.9, 9.0], 10.0, False),
])
def test_seen_enough(times, now, seen):
    assert bm.seen_enough(times, now) is seen


@pytest.mark.parametrize("custom,name", [
    (3 << 24 | 4 << 16, "Hold"), (6 << 24 | 4 << 16, "Land"), (5 << 24 | 4 << 16, "Return"),
    (2 << 24 | 4 << 16, "Takeoff"), (6 << 16, "Offboard"), (3 << 16, "Position"),
])
def test_px4_mode_name(custom, name):
    assert bm.px4_mode_name(custom) == name


# ---- refusals before anything starts

def test_plan_flag_prints_the_geometry_and_starts_nothing(capsys):
    assert bm.main(["--plan"]) == 0
    out = capsys.readouterr().out
    assert "distance 103.0 m" in out and "heading 8.6 deg" in out


@pytest.mark.parametrize("argv", [["--launch-xy=oops"], ["--launch-xy=0,0"],
                                  ["--person-xy=-101.87,-15.48"], ["--launch-xy=1,2,3"]])
def test_bad_geometry_is_refused(argv, capsys):
    assert bm.main(argv) == 2
    assert "error" in capsys.readouterr().err


def test_without_a_simulator_model_or_perception_it_refuses_to_start(monkeypatch, capsys):
    monkeypatch.delenv("HALO_TRITON_URL", raising=False)
    monkeypatch.delenv("PX4_SIM_MODEL", raising=False)
    assert bm.main([]) == 2
    monkeypatch.setenv("PX4_SIM_MODEL", "gz_x500_depth")
    assert bm.main([]) == 2
    assert "Perception" in capsys.readouterr().err
