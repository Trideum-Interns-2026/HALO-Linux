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
