"""Logic tests for the sim helper scripts (no sim, ROS or MAVLink needed)."""
import importlib.machinery
import importlib.util
import os

import pytest

SCRIPTS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def load(name, filename):
    loader = importlib.machinery.SourceFileLoader(name, os.path.join(SCRIPTS, filename))
    spec = importlib.util.spec_from_loader(name, loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


vehicle = load("sitl_vehicle", "sitl_vehicle")
check = load("sim_check", "sim_check.py")


@pytest.mark.parametrize("endpoint", ["tcp:127.0.0.1:5760", "udpin:0.0.0.0:14540",
                                      "udpout:localhost:14580"])
def test_local_endpoints_allowed(endpoint):
    vehicle.check_endpoint(endpoint)


@pytest.mark.parametrize("endpoint", ["udpin:192.168.1.5:14540", "serial:/dev/ttyACM0:57600",
                                      "udpin:example.com:1", "tcp:10.0.0.2:5760", "nonsense"])
def test_remote_or_serial_endpoints_refused(endpoint):
    with pytest.raises(ValueError):
        vehicle.check_endpoint(endpoint)


@pytest.mark.parametrize("alt", [0.1, 3.0, 10.0])
def test_takeoff_altitude_in_range(alt):
    assert vehicle.check_altitude(alt) == alt


@pytest.mark.parametrize("alt", [0.0, -1.0, 10.01, float("nan")])
def test_takeoff_altitude_out_of_range(alt):
    with pytest.raises(ValueError):
        vehicle.check_altitude(alt)


@pytest.mark.parametrize("custom_mode,name", [
    ((4 << 16) | (3 << 24), "Hold"), (6 << 16, "Offboard"), ((4 << 16) | (5 << 24), "Return"),
    ((4 << 16) | (2 << 24), "Takeoff"), ((4 << 16) | (6 << 24), "Land"), (3 << 16, "Position"),
    (0, "Unknown(0)")])
def test_px4_mode_names(custom_mode, name):
    assert vehicle.px4_mode_name(custom_mode) == name


def test_parse_position_round_trip():
    text = "pos_ned_m=1.50,-2.00,-3.00 vel_ned_mps=0.10,0.00,-0.20 yaw_deg=96.1"
    assert check.parse_position(text) == {"pos": [1.5, -2.0, -3.0], "vel": [0.1, 0.0, -0.2],
                                          "yaw": 96.1}
    assert check.parse_position("error: no LOCAL_POSITION_NED") is None


def test_local_distance():
    assert check.local_distance_m(47.0, 8.0, 47.0, 8.0) == 0.0
    assert check.local_distance_m(47.0, 8.0, 47.0001, 8.0) == pytest.approx(11.1, abs=0.1)


def write_log(tmp_path, lines):
    path = tmp_path / "movement.log"
    path.write_text("\n".join(lines))
    return str(path)


def test_logged_limits_at_the_limit_pass(tmp_path):
    path = write_log(tmp_path, [
        "target x=955.5 px, distance=9.52 m, forward=0.70 m/s, yaw=-45.0 deg/s",
        "Target lost; searching with spiral scan: forward=0.28 m/s, strafe=-0.45 m/s, yaw=25.0 deg/s"])
    ok, count, worst = check.logged_limits_ok(path)
    assert ok and count == 2 and worst["strafe"] == 0.45


@pytest.mark.parametrize("line", [
    "target x=1 px, distance=9 m, forward=0.80 m/s, yaw=0.0 deg/s",
    "Target lost; searching with spiral scan: forward=0.1 m/s, strafe=0.50 m/s, yaw=0.0 deg/s",
    "target x=1 px, distance=9 m, forward=0.1 m/s, yaw=46.0 deg/s"])
def test_logged_limits_exceeded_fail(tmp_path, line):
    assert not check.logged_limits_ok(write_log(tmp_path, [line]))[0]


def test_no_commands_is_not_ok(tmp_path):
    ok, count, _ = check.logged_limits_ok(write_log(tmp_path, ["Waiting for an armed vehicle"]))
    assert count == 0


def test_orbit_limit_is_separate_from_the_follow_limit(tmp_path):
    path = write_log(tmp_path, [
        "orbit target x=960.0 px, distance=6.00 m, forward=0.00 m/s, strafe=1.26 m/s, yaw=-12.0 deg/s"])
    assert not check.logged_limits_ok(path)[0]                       # over the follow strafe limit
    assert check.logged_limits_ok(path, check.LIMIT_ORBIT_STRAFE)[0]  # fine for orbit
    over = write_log(tmp_path, [
        "orbit target x=960.0 px, distance=6.00 m, forward=0.00 m/s, strafe=1.60 m/s, yaw=-12.0 deg/s"])
    assert not check.logged_limits_ok(over, check.LIMIT_ORBIT_STRAFE)[0]


def test_follow_log_lines_with_strafe_are_parsed(tmp_path):
    path = write_log(tmp_path, [
        "follow target x=1.0 px, distance=7.00 m, forward=0.35 m/s, strafe=0.16 m/s, yaw=22.5 deg/s"])
    ok, count, worst = check.logged_limits_ok(path)
    assert ok and count == 1 and worst["strafe"] == 0.16


def trace(vx_of_t, yaw_of_t=lambda t: 0.0, seconds=10.0, rate=10.0):
    return [{"t": i / rate, "x": 0.0, "y": 0.0, "z": -1.5, "vx": vx_of_t(i / rate), "vy": 0.0,
             "vz": 0.0, "yaw": yaw_of_t(i / rate)} for i in range(int(seconds * rate))]


def test_smoothness_of_steady_motion_is_zero():
    figures = check.smoothness(trace(lambda t: 1.0))
    assert figures["accel"][0] == 0.0 and figures["jerk"][1] == 0.0
    assert figures["yaw_rate"][0] == 0.0


def test_smoothness_sees_a_constant_acceleration_but_no_jerk():
    figures = check.smoothness(trace(lambda t: 0.5 * t))   # 0.5 m/s^2
    assert figures["accel"][0] == pytest.approx(0.5, abs=1e-6)
    assert figures["jerk"][1] == pytest.approx(0.0, abs=1e-6)


def test_a_step_in_velocity_is_a_jerk_and_a_ramp_is_not():
    step = check.smoothness(trace(lambda t: 0.0 if t < 5.0 else 1.0))
    ramp = check.smoothness(trace(lambda t: min(max(t - 4.0, 0.0), 2.0) * 0.5))
    assert step["jerk"][1] > 3 * ramp["jerk"][1]
    assert step["accel"][1] > 1.5 * ramp["accel"][1]


def test_yaw_rate_handles_the_wrap_at_180_degrees():
    figures = check.smoothness(trace(lambda t: 0.0,
                                     lambda t: ((20.0 * t + 180.0) % 360.0) - 180.0))
    assert figures["yaw_rate"][0] == pytest.approx(20.0, abs=1e-6)
    assert figures["yaw_accel"][1] == pytest.approx(0.0, abs=1e-6)


def test_short_or_missing_traces_give_no_figures(tmp_path):
    assert check.smoothness([]) is None
    assert check.smoothness(trace(lambda t: 0.0, seconds=1.0, rate=2.0)) is None
    assert check.read_trace(str(tmp_path / "missing.csv")) == []


def test_read_trace_parses_the_helper_output(tmp_path):
    path = tmp_path / "t.csv"
    path.write_text("t_s,x,y,z,vx,vy,vz,yaw_deg\n0.1,1,2,-3,0.1,0.2,0.3,45.0\nbroken\n")
    rows = check.read_trace(str(path))
    assert len(rows) == 1 and rows[0]["vy"] == 0.2 and rows[0]["yaw"] == 45.0


def test_read_trace_drops_rows_before_the_first_yaw(tmp_path):
    path = tmp_path / "t.csv"
    path.write_text("t_s,x,y,z,vx,vy,vz,yaw_deg\n0.1,1,2,-3,0.1,0.2,0.3,nan\n"
                    "0.2,1,2,-3,0.1,0.2,0.3,10.0\n")
    rows = check.read_trace(str(path))
    assert len(rows) == 1 and rows[0]["yaw"] == 10.0


# ---------------------------------------------------------------- goto / hold (sitl_vehicle)

@pytest.mark.parametrize("north,east,speed", [(0, 0, 1), (60, 0, 5), (0, -60, 8), (36, 48, 8),
                                              (-10.5, 3.2, 0.1)])
def test_goto_within_limits_is_accepted(north, east, speed):
    assert vehicle.check_goto(north, east, speed) == (north, east, speed)


@pytest.mark.parametrize("north,east,speed", [
    (60.01, 0, 5), (0, 60.01, 5), (43, 43, 5),                    # leg longer than MAX_GOTO_M
    (10, 10, 0), (10, 10, -1), (10, 10, 8.01),                    # speed outside (0, MAX]
    (float("nan"), 0, 5), (0, float("inf"), 5), (1, 1, float("nan")),
])
def test_goto_outside_limits_is_refused(north, east, speed):
    with pytest.raises(ValueError):
        vehicle.check_goto(north, east, speed)


def test_goto_limits_are_the_documented_ones():
    assert vehicle.MAX_GOTO_M == 60.0
    assert vehicle.MAX_GOTO_SPEED_MPS == 8.0
    assert 0 < vehicle.DEFAULT_GOTO_SPEED_MPS <= vehicle.MAX_GOTO_SPEED_MPS


@pytest.mark.parametrize("lat", [0.0, 37.4, 47.397971, -33.9])
def test_offset_latlon_round_trip_distance(lat):
    import math
    lat2, lon2 = vehicle.offset_latlon(lat, 8.5, 30.0, 40.0)
    north = math.radians(lat2 - lat) * vehicle.EARTH_RADIUS_M
    east = math.radians(lon2 - 8.5) * vehicle.EARTH_RADIUS_M * math.cos(math.radians(lat))
    assert north == pytest.approx(30.0, abs=1e-6)
    assert east == pytest.approx(40.0, abs=1e-6)


def test_offset_latlon_zero_is_identity_and_signs_point_the_right_way():
    assert vehicle.offset_latlon(47.4, 8.5, 0, 0) == (47.4, 8.5)
    lat, lon = vehicle.offset_latlon(47.4, 8.5, -10, -10)
    assert lat < 47.4 and lon < 8.5


# ---------------------------------------------------------------- sim_check --approach

def test_approach_geometry_at_the_chosen_bearing():
    import math
    g = check.approach_geometry(200.0, 200.0)
    x, y = g["drone_xy"]
    assert math.hypot(x, y) == pytest.approx(200.0)
    assert (x, y) == pytest.approx((-187.939, -68.404), abs=1e-3)
    # the person is at the world origin, so in the drone's NED frame it is (-y north, -x east)
    assert g["person_ned"] == pytest.approx((68.404, 187.939), abs=1e-3)
    assert math.hypot(*g["person_ned"]) == pytest.approx(200.0)


@pytest.mark.parametrize("bearing", [0, 45, 90, 135, 180, 200, 270, 315, -30])
def test_approach_geometry_faces_the_person(bearing):
    import math
    g = check.approach_geometry(150.0, bearing)
    x, y = g["drone_xy"]
    yaw = g["yaw_rad"]
    assert -math.pi <= yaw <= math.pi
    # the drone's nose (x forward at this yaw) points from the drone to the origin
    assert (math.cos(yaw), math.sin(yaw)) == pytest.approx((-x / 150.0, -y / 150.0), abs=1e-9)
    assert g["model_pose"].startswith("%.3f,%.3f,0,0,0," % (x, y))


def test_leg_offset_is_capped_and_points_at_the_person():
    import math
    dn, de, remaining = check.leg_offset((0.0, 0.0), (60.0, 80.0), 30.0)
    assert remaining == pytest.approx(100.0)
    assert math.hypot(dn, de) == pytest.approx(30.0)
    assert (dn, de) == pytest.approx((18.0, 24.0))


@pytest.mark.parametrize("pos,person", [((0, 0), (3, 4)), ((10, 10), (10, 10)), ((5, 5), (5, 25))])
def test_leg_offset_short_distances_go_all_the_way(pos, person):
    dn, de, remaining = check.leg_offset(pos, person, 30.0)
    assert (dn, de) == (person[0] - pos[0], person[1] - pos[1])
    assert remaining == pytest.approx((dn ** 2 + de ** 2) ** 0.5)


def test_every_approach_leg_is_within_sitl_vehicles_limit():
    assert check.APPROACH_LEG_M <= vehicle.MAX_GOTO_M
    assert 0 < check.APPROACH_SPEED_MPS <= vehicle.MAX_GOTO_SPEED_MPS
    assert check.APPROACH_GIVE_UP_M > check.FOLLOW_DISTANCE_M


@pytest.mark.parametrize("measured,true,ok", [
    (200.0, 200.0, True), (150.0, 200.0, True), (149.9, 200.0, False),   # 25 % of 200 m
    (10.0, 12.9, True), (10.0, 14.1, False),                             # 3 m floor at short range
    (0.0, 2.0, True), (40.0, 20.0, False),
])
def test_range_tolerance(measured, true, ok):
    assert check.range_ok(measured, true) is ok


@pytest.mark.parametrize("d,ok", [(6.0, True), (3.5, True), (8.5, True), (3.49, False),
                                  (8.51, False), (25.0, False)])
def test_follow_distance_tolerance(d, ok):
    assert check.follow_distance_ok(d) is ok


class _Args:
    def __init__(self, **kw):
        self.approach, self.world, self.approach_bearing = 200.0, "baylands", None
        self.orbit = self.views = self.skip_flight = False
        self.__dict__.update(kw)


@pytest.mark.parametrize("kw,fragment", [
    ({}, None),
    ({"approach": 0.0}, None),
    ({"approach": -1.0}, "--approach must be"),
    ({"approach": 300.01}, "--approach must be"),
    ({"world": "default"}, "only checked in baylands"),
    ({"world": "default", "approach_bearing": 30.0}, None),
    ({"orbit": True}, "--orbit"),
    ({"views": True}, "--views"),
    ({"skip_flight": True}, "--skip-flight"),
])
def test_check_approach_args(kw, fragment):
    problem = check.check_approach_args(_Args(**kw))
    if fragment is None:
        assert problem is None
    else:
        assert fragment in problem


def test_approach_without_perception_is_refused_before_anything_starts(monkeypatch):
    monkeypatch.delenv("HALO_TRITON_URL", raising=False)
    assert check.main(["--world", "baylands", "--approach", "200"]) == 2
    assert check.main(["--approach", "200"]) == 2          # default world: wrong direction data
