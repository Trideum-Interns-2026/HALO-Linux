#!/usr/bin/env python3
"""One-command end-to-end check of the whole HALO stack in the simulator.

    sim_check [--gui] [--watch] [--skip-flight]

Brings up PX4 SITL + Gazebo, the sim links (GPS + clock + uXRCE-DDS), Perception, Geolocation,
Tracking, then (unless --skip-flight) arms, takes off, runs Movement, checks it moved the vehicle
and handed back to Hold, lands, disarms, and tears everything down. Prints a PASS/FAIL table and
exits non-zero if anything FAILs. It is the only program besides `sitl_vehicle` allowed to arm
(AGENTS.md "Safety and hardware readiness"): SITL only, and it refuses to run if it cannot find the
simulator. Run it through `sim_check` (the bash wrapper sources ROS).

--gui     open the Gazebo window on DISPLAY=:0 (the Selkies desktop), via vglrun.
--orbit   Movement circles the person (30 s per lap) instead of following; checks that the nose
          turns with the circle.
--watch   implies --gui; pauses before arming and after landing and follows longer, so a human can
          watch.
"""

import argparse
import math
import os
import re
import signal
import subprocess
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOG_DIR = "/tmp/halo-check"
VEHICLE = os.path.join(REPO, "scripts", "sitl_vehicle")
SPAWN_PERSON = os.path.join(REPO, "Perception", "scripts", "spawn_person.sh")

WORLD_SERVICE = "/world/default/create"   # same service scripts/fly waits for
SIM_TIMEOUT_S = 90.0
TOPIC_TIMEOUT_S = 20.0
CAMERA_RATE_WINDOW_S = 10.0
CAMERA_RATE_PASS_HZ = 12.0
CAMERA_RATE_WARN_HZ = 5.0
TAKEOFF_ALT_M = 1.5       # low enough that a person 6 m away is not cut off at the bottom of the image
TAKEOFF_SETTLE_S = 10.0
LINK_TIMEOUT_S = 40.0
ARM_TIMEOUT_S = 30.0
TAKEOFF_MIN_ALT_M = 1.2
FOLLOW_S = 30.0
WATCH_FOLLOW_S = 90.0
WATCH_PRE_ARM_S = 60.0
WATCH_POST_LAND_S = 60.0
LAND_TIMEOUT_S = 40.0
MOTION_MIN_M = 0.3        # Movement must move the vehicle at least this far (or yaw below)...
MOTION_MIN_YAW_DEG = 5.0
STOP_HOLD_TIMEOUT_S = 8.0
ORBIT_PERIOD_S = 30.0
SMOOTH_WINDOW_S = 0.5     # spacing of the samples differentiated for the smoothness figures
TUNED_PARAMS = os.path.join(REPO, "Movement", "config", "sim_tuned.yaml")
GEOLOCATION_TUNED = os.path.join(REPO, "Geolocation", "config", "tuned.yaml")
MAX_SPEED = 5.0
VIEW_SETTLE_S = 2.5
VIEW_WINDOW_S = 5.0
VIEW_MIN_CONFIDENCE = 0.05
# px4 -d = daemon mode, no interactive shell. Without it PX4 reads EOF from the closed stdin and
# spins printing prompts: ~1 busy core and a 900 MB log per run.
PX4_ARGS = ["-d"]
ORBIT_S = 35.0            # a bit more than one lap
ORBIT_MIN_COMMANDS = 10
ORBIT_MIN_YAW_DEG = 90.0  # at 360 deg per lap, 90 deg proves the nose is turning with the circle
# Movement's documented command limits (see its startup log line / Movement/README.md).
LIMIT_FORWARD = 0.70
LIMIT_STRAFE = 0.45
LIMIT_ORBIT_STRAFE = 1.5   # Movement's orbit-only speed cap (max_orbit_speed_m_s default)
LIMIT_YAW_DEG_S = 45.0
LIMIT_SLACK = 1.001
LEFTOVER_PATTERN = r"px4|gz sim|MicroXRCE|parameter_bridge|ros2 launch|yolo_person|tracking_node|" \
                   r"geolocation_node|movement_node|sitl_vehicle (link|trace)"

SPEED = 1.0           # --speed: Gazebo real-time factor requested through PX4_SIM_SPEED_FACTOR
results = []          # (name, status, detail)
procs = {}            # name -> Popen, each in its own session / process group


def record(name, status, detail=""):
    results.append((name, status, detail))
    print("  [%-4s] %s  %s" % (status, name, detail), flush=True)


def log(msg):
    print("== " + msg, flush=True)


def start(name, cmd, env=None):
    os.makedirs(LOG_DIR, exist_ok=True)
    fh = open(os.path.join(LOG_DIR, name + ".log"), "w")
    full_env = dict(os.environ)
    full_env.update(env or {})
    procs[name] = subprocess.Popen(cmd, stdout=fh, stderr=subprocess.STDOUT, env=full_env,
                                   start_new_session=True)
    return procs[name]


def stop(name, grace_s=8.0):
    proc = procs.pop(name, None)
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGTERM)
        proc.wait(timeout=grace_s)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass


def run(cmd, timeout=30.0):
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return out.returncode, (out.stdout + out.stderr).strip()
    except subprocess.TimeoutExpired:
        return 124, "timed out"


def vehicle(*args, timeout=40.0):
    return run([VEHICLE] + list(args), timeout=timeout)


def parse_position(text):
    m = re.search(r"pos_ned_m=([-\d.]+),([-\d.]+),([-\d.]+) vel_ned_mps=([-\d.]+),([-\d.]+),"
                  r"([-\d.]+) yaw_deg=([-\d.naN]+)", text)
    if not m:
        return None
    v = [float(x) for x in m.groups()]
    return {"pos": v[0:3], "vel": v[3:6], "yaw": v[6]}


def read_trace(path):
    """Rows of `sitl_vehicle trace` CSV as dicts of floats; empty when missing or malformed."""
    rows = []
    try:
        with open(path) as fh:
            next(fh, None)
            for line in fh:
                parts = line.strip().split(",")
                if len(parts) == 8:
                    values = [float(v) for v in parts]
                    if all(math.isfinite(v) for v in values):   # drops rows before the first yaw
                        rows.append(dict(zip(("t", "x", "y", "z", "vx", "vy", "vz", "yaw"),
                                             values)))
    except (OSError, ValueError):
        return []
    return rows


def smoothness(rows, window_s=SMOOTH_WINDOW_S):
    """Motion smoothness of a flight trace: horizontal acceleration and jerk, yaw rate and yaw
    acceleration, each as (rms, peak). Differences are taken `window_s` apart so sensor noise at
    PX4's stream rate does not dominate. None when the trace is too short."""
    picked = []
    for row in rows:
        if not picked or row["t"] - picked[-1]["t"] >= window_s:
            picked.append(row)
    if len(picked) < 4:
        return None
    acc, yaw_rate = [], []
    for a, b in zip(picked, picked[1:]):
        dt = b["t"] - a["t"]
        acc.append(((b["vx"] - a["vx"]) / dt, (b["vy"] - a["vy"]) / dt))
        dyaw = (b["yaw"] - a["yaw"] + 180.0) % 360.0 - 180.0
        yaw_rate.append(dyaw / dt)
    dts = [b["t"] - a["t"] for a, b in zip(picked[1:], picked[2:])]
    jerk = [math.hypot(b[0] - a[0], b[1] - a[1]) / dt
            for a, b, dt in zip(acc, acc[1:], dts)]
    yaw_acc = [(b - a) / dt for a, b, dt in zip(yaw_rate, yaw_rate[1:], dts)]

    def stat(values):
        return (math.sqrt(sum(v * v for v in values) / len(values)),
                max(abs(v) for v in values))

    return {"accel": stat([math.hypot(*a) for a in acc]), "jerk": stat(jerk),
            "yaw_rate": stat(yaw_rate), "yaw_accel": stat(yaw_acc)}


def vehicle_state():
    """(armed, mode) from `sitl_vehicle status`, or None."""
    rc, out = vehicle("status", timeout=15.0)
    m = re.match(r"(armed|disarmed) mode=(\S+)", out)
    return (m.group(1) == "armed", m.group(2)) if m and rc == 0 else None


def wait_for(predicate, timeout_s, period_s=1.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(period_s)
    return False


def world_up():
    rc, out = run(["gz", "service", "-l"], timeout=10.0)
    return rc == 0 and WORLD_SERVICE in out.split()


def window_up():
    rc, out = run(["xwininfo", "-root", "-tree"], timeout=10.0)
    return rc == 0 and "gazebo" in out.lower()


def leftovers():
    _, out = run(["pgrep", "-af", LEFTOVER_PATTERN], timeout=10.0)
    mine = {str(os.getpid()), str(os.getppid())}
    return [ln for ln in out.splitlines()
            if ln.split(" ", 1)[0] not in mine and "sim_check" not in ln and "pgrep" not in ln]


def local_distance_m(lat1, lon1, lat2, lon2):
    """Flat-earth horizontal distance, good for the tens of metres compared here."""
    dn = math.radians(lat2 - lat1) * 6371000.0
    de = math.radians(lon2 - lon1) * 6371000.0 * math.cos(math.radians(lat1))
    return math.hypot(dn, de)


def logged_limits_ok(log_path, strafe_limit=LIMIT_STRAFE):
    """Every command Movement logged must respect its limits. Returns (ok, n_commands, worst)."""
    worst = {"forward": 0.0, "strafe": 0.0, "yaw": 0.0}
    count = 0
    pattern = re.compile(r"forward=(-?[\d.]+) m/s(?:, strafe=(-?[\d.]+) m/s)?, yaw=(-?[\d.]+) deg/s")
    try:
        with open(log_path, errors="replace") as fh:
            for line in fh:
                m = pattern.search(line)
                if not m:
                    continue
                count += 1
                worst["forward"] = max(worst["forward"], abs(float(m.group(1))))
                worst["strafe"] = max(worst["strafe"], abs(float(m.group(2) or 0.0)))
                worst["yaw"] = max(worst["yaw"], abs(float(m.group(3))))
    except OSError:
        return False, 0, worst
    ok = (worst["forward"] <= LIMIT_FORWARD * LIMIT_SLACK
          and worst["strafe"] <= strafe_limit * LIMIT_SLACK
          and worst["yaw"] <= LIMIT_YAW_DEG_S * LIMIT_SLACK)
    return ok, count, worst


class Probe:
    """Small rclpy helper: wait for one message, or count messages for a while."""

    def __init__(self):
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        self.rclpy = rclpy
        self.qos = qos_profile_sensor_data   # best-effort: compatible with any publisher
        rclpy.init()
        self.node = rclpy.create_node("sim_check_probe")

    def one(self, msg_type, topic, timeout_s=TOPIC_TIMEOUT_S, accept=None):
        from rclpy.wait_for_message import wait_for_message
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            ok, msg = wait_for_message(msg_type, self.node, topic, qos_profile=self.qos,
                                       time_to_wait=min(2.0, max(0.1, deadline - time.monotonic())))
            if ok and (accept is None or accept(msg)):
                return msg
        return None

    def collect(self, msg_type, topic, window_s):
        """Every message received on `topic` during `window_s` seconds."""
        got = []
        sub = self.node.create_subscription(msg_type, topic, got.append, self.qos)
        end = time.monotonic() + window_s
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.destroy_subscription(sub)
        return got

    def rate(self, msg_type, topic, window_s):
        count = [0]
        sub = self.node.create_subscription(
            msg_type, topic, lambda _m: count.__setitem__(0, count[0] + 1), self.qos)
        end = time.monotonic() + window_s
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.destroy_subscription(sub)
        return count[0] / window_s

    def close(self):
        try:
            self.node.destroy_node()
            self.rclpy.shutdown()
        except Exception:
            pass


def speed_env():
    return {"PX4_SIM_SPEED_FACTOR": "%g" % SPEED} if SPEED != 1.0 else {}


def check_views(probe):
    """How well the detector sees the person from each side, with the threshold lowered so
    near-misses show. The person is 6 m ahead; yaw 0 faces the drone (spawn_person.sh)."""
    from perception_msgs.msg import Detection
    for label, yaw in (("front", 0.0), ("left side", math.pi / 2), ("back", math.pi),
                       ("right side", -math.pi / 2)):
        run([SPAWN_PERSON, "6", "0", "%.4f" % yaw], timeout=30.0)
        time.sleep(VIEW_SETTLE_S)
        got = probe.collect(Detection, "/perception/detection", VIEW_WINDOW_S)
        confs = [d.confidence for d in got]
        if confs:
            detail = "%d detections in %.0f s, confidence min %.2f mean %.2f max %.2f" % (
                len(confs), VIEW_WINDOW_S, min(confs), sum(confs) / len(confs), max(confs))
        else:
            detail = "no detection in %.0f s even at threshold %.2f" % (VIEW_WINDOW_S,
                                                                        VIEW_MIN_CONFIDENCE)
        record("view_" + label.replace(" ", "_"),
               "PASS" if confs and min(confs) >= 0.5 else "WARN" if confs else "FAIL", detail)
    run([SPAWN_PERSON, "6", "0", "0"], timeout=30.0)   # put the person back facing the drone


def gz_rtf():
    """Gazebo's current real-time factor from its /stats topic, or None."""
    _, out = run(["gz", "topic", "-e", "-t", "/stats", "-n", "1"], timeout=15.0)
    m = re.search(r"real_time_factor:\s*([\d.]+)", out)
    return float(m.group(1)) if m else None


def gz_camera_rate():
    """Raw last line of `gz topic -f` for the camera, to tell Gazebo-side from ROS-side drops."""
    _, topics = run(["gz", "topic", "-l"], timeout=10.0)
    cams = sorted(t for t in topics.split() if re.match(r"^/world/[^/]+/model/.+/sensor/[^/]+/image$", t))
    if not cams:
        return "gz camera topic not found"
    _, out = run(["timeout", "8", "gz", "topic", "-f", "-t", cams[0]], timeout=15.0)
    return (out.splitlines() or ["no output"])[-1][:80]


def check_services(probe, have_triton):
    from geolocation_msgs.msg import GeolocationState
    from perception_msgs.msg import Detection
    from px4_msgs.msg import VehicleAttitude
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Image, NavSatFix
    from tracking_msgs.msg import TrackingState

    msg = probe.one(Clock, "/clock", 10.0)
    record("clock", "PASS" if msg else "FAIL",
           "sim time %d s" % msg.clock.sec if msg else "no /clock (sim_links bridge)")

    msg = probe.one(NavSatFix, "/gps/fix")
    record("gps", "PASS" if msg and math.isfinite(msg.latitude) else "FAIL",
           "lat %.6f lon %.6f" % (msg.latitude, msg.longitude) if msg else "no /gps/fix")

    geo = probe.one(GeolocationState, "/geolocation/position")
    if geo and all(math.isfinite(v) and v >= 0 for v in
                   (geo.error_x_m, geo.error_y_m, geo.error_z_m)):
        record("geolocation", "PASS", "error xyz %.2f %.2f %.2f m, stamp %d.%03d"
               % (geo.error_x_m, geo.error_y_m, geo.error_z_m, geo.header.stamp.sec,
                  geo.header.stamp.nanosec // 1000000))
    else:
        record("geolocation", "FAIL", "no valid /geolocation/position")

    msg = probe.one(VehicleAttitude, "/fmu/out/vehicle_attitude", 10.0)
    record("px4_attitude", "PASS" if msg else "FAIL",
           "uXRCE-DDS publishing" if msg else "no /fmu/out/vehicle_attitude (agent not running?)")

    if not have_triton:
        record("perception", "SKIP", "HALO_TRITON_URL not set (Triton is Spark-only)")
        record("tracking", "SKIP", "needs perception")
        return
    det = probe.one(Detection, "/perception/detection", accept=lambda d: d.confidence > 0.5)
    record("perception", "PASS" if det else "FAIL",
           "confidence %.2f box x %.0f-%.0f" % (det.confidence, det.x_min, det.x_max)
           if det else "no detection of the spawned person")
    trk = probe.one(TrackingState, "/tracking/position", 30.0)
    if trk and geo and all(math.isfinite(v) for v in (trk.latitude_deg, trk.longitude_deg)):
        dist = local_distance_m(geo.latitude_deg, geo.longitude_deg,
                                trk.latitude_deg, trk.longitude_deg)
        record("tracking", "PASS" if 0.5 < dist < 30.0 else "FAIL",
               "person %.1f m from the drone, error xyz %.2f %.2f %.2f m"
               % (dist, trk.error_x_m, trk.error_y_m, trk.error_z_m))
    else:
        record("tracking", "FAIL", "no /tracking/position (see %s/tracking.log)" % LOG_DIR)

    hz = probe.rate(Image, "/camera/front/image_raw", CAMERA_RATE_WINDOW_S)
    # The camera ticks in sim time, so at speed N it should arrive N times faster in wall time.
    status = ("PASS" if hz >= CAMERA_RATE_PASS_HZ * SPEED else
              "WARN" if hz >= CAMERA_RATE_WARN_HZ * SPEED else "FAIL")
    record("camera_rate", status, "%.1f Hz best-effort (sim camera is %.0f Hz wall); gazebo side: %s"
           % (hz, 15.0 * SPEED, gz_camera_rate()))
    rtf = gz_rtf()
    record("sim_speed", "SKIP" if rtf is None else "PASS" if rtf >= 0.9 * SPEED else "WARN",
           "real-time factor %s (asked for %.1f)" % ("unknown" if rtf is None else "%.2f" % rtf,
                                                     SPEED))


class Flight:
    """What this run did to the vehicle, so it can always be put back on the ground."""
    armed = False
    start_pose = None


def takeoff(args, fl):
    """Check the MAVLink link, arm and climb. Returns True when hovering at the target height."""
    state = None

    def link_up():
        nonlocal state
        state = vehicle_state()
        return state is not None

    wait_for(link_up, LINK_TIMEOUT_S, 2.0)
    record("vehicle_link", "PASS" if state else "FAIL",
           "%s mode=%s" % ("armed" if state[0] else "disarmed", state[1]) if state
           else "sitl_vehicle cannot talk to PX4 (MAVLink TCP bridge on :5760 up?)")
    if not state:
        return False
    if args.watch:
        log("watch mode: %d s before arming" % WATCH_PRE_ARM_S)
        time.sleep(WATCH_PRE_ARM_S)
    # PX4 refuses to arm until its sensors and position estimate are ready, which can take a few
    # seconds after boot, so retry rather than sleep.
    result = {}

    def armed_ok():
        result["rc"], result["out"] = vehicle("arm")
        return result["rc"] == 0

    fl.armed = wait_for(armed_ok, ARM_TIMEOUT_S, 2.0)
    rc1, out1 = result["rc"], result["out"]
    rc2, out2 = vehicle("takeoff", str(TAKEOFF_ALT_M)) if fl.armed else (1, "not armed")
    pos = None

    def climbed():
        nonlocal pos
        pos = parse_position(vehicle("position")[1])
        return pos is not None and -pos["pos"][2] >= TAKEOFF_MIN_ALT_M

    reached = fl.armed and rc2 == 0 and wait_for(climbed, TAKEOFF_SETTLE_S, 1.0)
    alt = -pos["pos"][2] if pos else 0.0
    record("arm_takeoff", "PASS" if reached else "FAIL",
           "arm: %s | takeoff: %s | altitude %.1f m" % (out1, out2, alt))
    fl.start_pose = pos
    return bool(reached)


def movement_phase(args, fl):
    """Run Movement (follow or orbit) for a while and check what it did to the vehicle."""
    mode = ("orbit" if args.orbit else "follow") + (", tuned gains" if args.tuned else "")
    cmd = ["ros2", "launch", "movement", "movement.launch.py"]
    if args.tuned:
        cmd.append("params_file:=" + TUNED_PARAMS)
    if args.orbit:
        cmd += ["orbit:=true", "orbit_period_s:=%g" % ORBIT_PERIOD_S]
    start("movement", cmd)
    trace_path = os.path.join(LOG_DIR, "trace.log")   # start() names logs <name>.log
    trace_s = (WATCH_FOLLOW_S if args.watch else (ORBIT_S if args.orbit else FOLLOW_S)) / SPEED
    start("trace", [VEHICLE, "trace", "%g" % (trace_s + 4.0)])
    movement_log = os.path.join(LOG_DIR, "movement.log")
    duration = WATCH_FOLLOW_S if args.watch else (ORBIT_S if args.orbit else FOLLOW_S)
    duration /= SPEED       # the lap and the follow are sim-time motions
    interval = (2.5 if args.orbit else 5.0) / SPEED
    time.sleep(2.0)
    start_pose = parse_position(vehicle("position")[1]) or fl.start_pose
    end_pose = last = start_pose
    yaw_total = 0.0       # unwrapped yaw change, deg: shows the nose turning with the circle
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        time.sleep(interval)
        cur = parse_position(vehicle("position")[1])
        if not cur:
            continue
        yaw_total += (cur["yaw"] - last["yaw"] + 180.0) % 360.0 - 180.0
        last = end_pose = cur
        if args.watch:
            print("   pos %s yaw %.0f" % (["%.1f" % v for v in cur["pos"]], cur["yaw"]), flush=True)
    with open(movement_log, errors="replace") as fh:
        text = fh.read()
    started = "Offboard mode active" in text
    record("movement_offboard", "PASS" if started else "FAIL",
           "offboard started (%s)" % mode if started else
           (re.findall(r"Waiting for an armed.*|Offboard.*failed.*", text) or ["no log"])[-1])
    moved = math.dist(start_pose["pos"][:2], end_pose["pos"][:2])
    record("movement_motion", "PASS" if moved >= MOTION_MIN_M or abs(yaw_total) >= MOTION_MIN_YAW_DEG
           else "FAIL", "moved %.2f m, yawed %.1f deg in %.0f s" % (moved, yaw_total, duration))
    if args.orbit:
        commands = len(re.findall(r"orbit target x=", text))
        lost = len(re.findall(r"Target lost", text))
        record("movement_orbit",
               "PASS" if commands >= ORBIT_MIN_COMMANDS and abs(yaw_total) >= ORBIT_MIN_YAW_DEG
               else "FAIL",
               "%d orbit commands, %d target-lost scans, nose turned %.0f deg in %.0f s "
               "(a full lap is 360)" % (commands, lost, yaw_total, duration))
    figures = smoothness(read_trace(trace_path))
    if figures:
        record("movement_smooth", "INFO",
               "accel rms %.2f (peak %.2f) m/s2 | jerk rms %.2f (peak %.2f) m/s3 | yaw rate rms "
               "%.1f deg/s | yaw accel rms %.1f (peak %.1f) deg/s2 [%s]"
               % (figures["accel"] + figures["jerk"] + (figures["yaw_rate"][0],)
                  + figures["yaw_accel"] + (mode,)))
    else:
        record("movement_smooth", "WARN", "no flight trace recorded (see %s)" % trace_path)
    ok, count, worst = logged_limits_ok(movement_log, LIMIT_ORBIT_STRAFE if args.orbit
                                        else LIMIT_STRAFE)
    record("movement_limits", "PASS" if ok and count else "FAIL" if not ok else "WARN",
           "%d commands, max fwd %.2f / strafe %.2f m/s, yaw %.1f deg/s"
           % (count, worst["forward"], worst["strafe"], worst["yaw"]))

    stop("movement")
    stop("trace")
    got_hold = wait_for(lambda: (vehicle_state() or (None, ""))[1] == "Hold",
                        STOP_HOLD_TIMEOUT_S, 1.0)
    record("movement_stop_hold", "PASS" if got_hold else "FAIL",
           "PX4 in Hold after Movement stopped" if got_hold else
           "mode after stop: %s" % (vehicle_state(),))


def land(args, fl):
    """Always runs after takeoff, whatever happened in between."""
    stop("movement")
    if not fl.armed:
        return
    vehicle("land")
    landed = wait_for(lambda: (parse_position(vehicle("position")[1]) or
                               {"pos": [0, 0, -99]})["pos"][2] > -0.3, LAND_TIMEOUT_S, 2.0)
    vehicle("disarm")
    final = vehicle_state()
    record("land_disarm", "PASS" if landed and final and not final[0] else "FAIL",
           "landed=%s, %s" % (landed, "disarmed" if final and not final[0] else final))
    fl.armed = False
    if args.watch:
        log("watch mode: %d s with the GUI still up" % WATCH_POST_LAND_S)
        time.sleep(WATCH_POST_LAND_S)


def teardown():
    log("teardown")
    for name in ("movement", "trace", "link", "tracking", "geolocation", "perception", "sim_links", "fly"):
        stop(name)
    run(["ros2", "daemon", "stop"], timeout=15.0)
    time.sleep(2.0)
    left = leftovers()
    record("teardown", "PASS" if not left else "FAIL",
           "no processes left" if not left else "left: " + "; ".join(l[:60] for l in left[:3]))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--gui", action="store_true")
    ap.add_argument("--watch", action="store_true")
    ap.add_argument("--skip-flight", action="store_true")
    ap.add_argument("--orbit", action="store_true",
                    help="Movement orbits the person (%.0f s per lap) instead of following"
                    % ORBIT_PERIOD_S)
    ap.add_argument("--speed", type=float, default=1.0,
                    help="run Gazebo at this real-time factor (PX4_SIM_SPEED_FACTOR); durations "
                         "shrink to match. The camera and Perception run in wall time, so expect "
                         "dropped frames above ~2")
    ap.add_argument("--tuned", action="store_true",
                    help="load the tuned parameter files: Movement/config/sim_tuned.yaml and "
                         "Geolocation/config/tuned.yaml")
    ap.add_argument("--views", action="store_true",
                    help="diagnostic, no flight: spawn the person facing four ways and report the "
                         "detector's confidence for each (threshold lowered to %.2f)"
                    % VIEW_MIN_CONFIDENCE)
    ap.add_argument("--no-link", action="store_true",
                    help="diagnostic: do not run the background GCS heartbeat (PX4 may fail over)")
    args = ap.parse_args(argv)
    args.gui = args.gui or args.watch
    global SPEED
    SPEED = min(max(args.speed, 0.1), MAX_SPEED)
    have_triton = bool(os.environ.get("HALO_TRITON_URL"))

    if world_up():
        print("error: a simulator is already running; stop it first (refusing to share it)",
              file=sys.stderr)
        return 2
    if leftovers():
        print("error: leftover sim processes exist:\n" + "\n".join(leftovers()), file=sys.stderr)
        return 2

    probe = None
    flight = Flight()
    t0 = time.monotonic()
    try:
        log("starting the simulator (%s)" % ("GUI" if args.gui else "headless"))
        if args.gui:
            start("fly", ["vglrun", "-d", "egl", os.path.join(REPO, "scripts", "fly")] + PX4_ARGS,
                  dict(speed_env(), DISPLAY=":0", VGL_DISPLAY="egl"))
        else:
            start("fly", [os.path.join(REPO, "scripts", "fly")] + PX4_ARGS,
                  dict(speed_env(), HEADLESS="1"))
        up = wait_for(world_up, SIM_TIMEOUT_S)
        record("sim_up", "PASS" if up else "FAIL",
               "world ready after %.0f s" % (time.monotonic() - t0) if up else
               "no %s within %.0f s (see %s/fly.log)" % (WORLD_SERVICE, SIM_TIMEOUT_S, LOG_DIR))
        if not up:
            return 1
        if args.gui:
            shown = wait_for(window_up, 30.0)
            record("gui", "PASS" if shown else "FAIL",
                   "Gazebo window on :0" if shown else "no Gazebo window (is the Selkies display up?)")

        log("starting sim links and services")
        start("sim_links", [os.path.join(REPO, "scripts", "sim_links")])
        start("geolocation", ["ros2", "launch", "geolocation", "geolocation.launch.py"]
              + (["params_file:=" + GEOLOCATION_TUNED] if args.tuned else []))
        start("tracking", ["ros2", "launch", "tracking", "tracking.launch.py",
                           "use_sim_time:=true"])
        if have_triton:
            start("perception", ["ros2", "launch", "perception", "perception.launch.py"]
                  + (["min_confidence:=%g" % VIEW_MIN_CONFIDENCE] if args.views else []))
        if not args.no_link:
            start("link", [VEHICLE, "link"])    # keeps a ground station visible to PX4
        run([SPAWN_PERSON, "6", "0", "0"], timeout=30.0)   # 6 m ahead = the follow distance

        flying = not args.skip_flight and not args.views and have_triton
        if not flying:
            record("flight", "SKIP", "--skip-flight" if args.skip_flight or args.views else
                   "Movement needs Perception detections (no Triton)")
        # Take off first and check the services while hovering, so nothing waits on the ground.
        airborne = False
        if flying:
            log("takeoff")
            airborne = takeoff(args, flight)

        log("checking services")
        probe = Probe()
        check_services(probe, have_triton)
        if args.views and have_triton:
            check_views(probe)
        probe.close()
        probe = None

        if airborne:
            log("movement (%s)" % ("orbit" if args.orbit else "follow"))
            movement_phase(args, flight)
    except KeyboardInterrupt:
        record("run", "FAIL", "interrupted")
    finally:
        if probe:
            probe.close()
        land(args, flight)
        if "fly" in procs and (vehicle_state() or (False, ""))[0]:   # never leave it armed
            vehicle("land")
            time.sleep(10.0)
            vehicle("disarm")
        teardown()

    print("\n%-20s %-5s %s" % ("CHECK", "", "DETAIL"))
    for name, status, detail in results:
        print("%-20s %-5s %s" % (name, status, detail))
    failed = [r for r in results if r[1] == "FAIL"]
    print("\n%d checks, %d FAIL, %d WARN, %d SKIP  (logs: %s, %.0f s)"
          % (len(results), len(failed), sum(r[1] == "WARN" for r in results),
             sum(r[1] == "SKIP" for r in results), LOG_DIR, time.monotonic() - t0))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
