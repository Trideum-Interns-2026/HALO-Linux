#!/usr/bin/env python3
"""Baylands mission, flown the way a human pilot would, SITL only. Everything is in this file.

    python3 scripts/baylands_mission.py [--headless] [--linger S] [--plan]
        [--launch-xy=X,Y] [--person-xy=X,Y]

Documented purpose (AGENTS.md, "Safety and hardware readiness"): this program brings up the
simulator in the Baylands world with the drone at the launch point, then arms it, takes off,
flies to the person, descends, switches on Movement's automatic follow, lands and shuts the
simulator down. It only runs against the simulator, on a command from a human or from the
sim-operator agent. Never run it against a real vehicle.

The mission, as a pilot would fly it:
  1. Start the sim with the drone at the launch point (default: the spot the user flew from,
     103 m from the person) and the person at the world origin, facing each other. Start
     Perception, Geolocation, Tracking and the ui bridge. Open the Gazebo window and the
     Perception view unless --headless.
  2. Pre-flight: wait for the position estimate, then arm (PX4 refuses until it is ready).
  3. Climb to 20 ft and fly towards the person in short legs, watching the camera (Perception).
     The pilot stays at 20 ft all the way and never climbs over trees: the route avoids them.
  4. Keep going after the first sighting until the person is ENGAGE_DISTANCE_M away, then hold
     and check that Perception still sees them and that Tracking agrees about where they are.
  5. Switch on Movement (the automatic follow), asked to follow at FOLLOW_DISTANCE_M. At 20 ft
     the camera only keeps the person's feet in frame beyond about 16 m, so the follow distance
     is set from the height (a parameter file, no Movement code is touched). If Tracking loses
     the person for LOST_ABORT_S the pilot takes over instead of letting Movement circle.
  6. Stop Movement (PX4 goes back to Hold), land, disarm, shut everything down.

Safety, all enforced in code: every command is bounded (MAX_ALT_M, MAX_LEG_M, MAX_SPEED_MPS, total
distance and time); on any failure, Ctrl+C or timeout the drone holds, lands and disarms, and the
simulator is torn down. The MAVLink link is the local TCP bridge only (tcp:127.0.0.1:5760).

The flight line from the default launch point to the person has no tree or building within 10 m
(checked on the Baylands mesh), so the 20 ft cruise never needs to climb over anything; the trees
near the launch point stand north and west of it, up to about 14 m tall. If you change
--launch-xy or --person-xy, check the line yourself: nothing in this file avoids a tree.

World frame: Gazebo's, x east, y north, metres. Local frame (what PX4 reports): NED from the
launch point.
"""

import argparse
import math
import os
import signal
import subprocess
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOG_DIR = "/tmp/halo-mission"
FLY = os.path.join(REPO, "scripts", "fly")
SIM_LINKS = os.path.join(REPO, "scripts", "sim_links")
SPAWN_PERSON = os.path.join(REPO, "Perception", "scripts", "spawn_person.sh")

# ---- where things are -------------------------------------------------------------------------
WORLD = "baylands"
LAUNCH_XY = (-101.87, -15.48)    # where the user's drone sat on the ground, world metres
PERSON_XY = (0.0, 0.0)           # the default spawn, solid ground
MAX_MISSION_M = 300.0            # launch point to person, at most
MIN_MISSION_M = 40.0

# ---- limits, every one enforced ---------------------------------------------------------------
CRUISE_ALT_M = 6.096             # 20 ft: below the treetops, so the route goes around trees
MAX_ALT_M = 10.0
MIN_ALT_M = 1.2
MAX_LEG_M = 30.0                 # one fly-to-point leg (sim helper limit is 60)
MAX_SPEED_MPS = 5.0              # (sim helper limit is 8)
ARRIVE_M = 3.0                   # a leg is done this close to its end
ENGAGE_DISTANCE_M = 25.0         # get this close before switching on Movement
STALL_S = 60.0                   # wall seconds without 0.5 m of progress on a leg: abort
APPROACH_TOTAL_S = 1500.0        # wall-clock limit for the approach
MAX_FLOWN_M = 400.0              # total horizontal distance the approach may fly

# ---- timing (wall-clock seconds; the sim can run well below real time) -------------------------
WORLD_WAIT_S = 300.0             # baylands loads big meshes
ARM_WAIT_S = 90.0
CLIMB_WAIT_S = 180.0
STOP_WAIT_S = 30.0
FOLLOW_TIMEOUT_S = 900.0
LAND_WAIT_S = 240.0
HEARTBEAT_PERIOD_S = 1.0
ACK_TIMEOUT_S = 5.0
SEEN_CONFIDENCE = 0.5
SEEN_COUNT = 2                   # detections in SEEN_WINDOW_S that count as "seen"
SEEN_WINDOW_S = 2.0
TRACKING_WAIT_S = 30.0
RANGE_TOL_FRACTION = 0.25        # Tracking's range may be this fraction off...
RANGE_TOL_MIN_M = 3.0            # ...but never held tighter than this
FOLLOW_DISTANCE_M = 20.0         # asked of Movement; its default 6 m puts the feet out of frame
MOVEMENT_MIN_DISTANCE_M = 14.0   # Movement's clamp band around it (hard caps are 2 and 30 m)
MOVEMENT_MAX_DISTANCE_M = 30.0
# tan of the largest angle below the camera axis at which the feet stay in frame: the camera's
# vertical field of view is 42 deg (half 21.2 deg) less Tracking's 12 px edge margin = 20.8 deg
FEET_IN_FRAME = 0.38
LOST_ABORT_S = 20.0              # no Tracking message this long while Movement runs: abort
TRACKING_FRESH_S = 2.0           # a Tracking message this recent counts as "Tracking sees them"
REDETECT_WAIT_S = 30.0           # at the engage point, wait this long for Perception to see them
MOVEMENT_PARAMS = os.path.join(LOG_DIR, "movement_params.yaml")
FOLLOW_TOL_M = 4.0
FOLLOW_STABLE_S = 10.0           # inside the follow band this long counts as followed
LANDED_Z_M = -0.3                # NED z above this: on the ground

ENDPOINT = ("127.0.0.1", 5760)   # the local MAVLink TCP bridge; nothing else is ever used
GCS_SYSTEM_ID, GCS_COMPONENT_ID = 255, 190
EARTH_RADIUS_M = 6378137.0
LEFTOVER = (r"px4|gz sim|MicroXRCE|parameter_bridge|ros2 launch|yolo_person|detection_overlay|"
            r"tracking_node|geolocation_node|movement_node|ui_bridge|debug_view")

PX4_MAIN_MODES = {1: "Manual", 2: "Altitude", 3: "Position", 5: "Acro", 6: "Offboard",
                  7: "Stabilized"}
PX4_AUTO_MODES = {1: "Ready", 2: "Takeoff", 3: "Hold", 4: "Mission", 5: "Return", 6: "Land"}


# ================================================================================================
# Pure functions (unit-tested, no sim, ROS or MAVLink needed)
# ================================================================================================

def parse_xy(text):
    """'X,Y' (metres) -> (x, y); ValueError when it is not two finite numbers."""
    parts = text.split(",")
    if len(parts) != 2:
        raise ValueError("expected X,Y, got %r" % text)
    x, y = float(parts[0]), float(parts[1])
    if not (math.isfinite(x) and math.isfinite(y)):
        raise ValueError("expected finite numbers, got %r" % text)
    return x, y


def plan(launch_xy, person_xy):
    """The mission geometry in Gazebo's world frame.

    The drone spawns at launch_xy facing the person, who faces the drone (the person model looks
    along -x at yaw 0, so both yaws are the direction from the drone to the person). person_ned
    is the person's position in the drone's local NED frame: (north, east) from the launch point.
    Raises ValueError when the mission distance is outside (MIN_MISSION_M, MAX_MISSION_M].
    """
    dx, dy = person_xy[0] - launch_xy[0], person_xy[1] - launch_xy[1]
    distance = math.hypot(dx, dy)
    if not MIN_MISSION_M <= distance <= MAX_MISSION_M:
        raise ValueError("launch point and person must be %.0f to %.0f m apart (got %.1f m)"
                         % (MIN_MISSION_M, MAX_MISSION_M, distance))
    yaw = math.atan2(dy, dx)
    return {"launch_xy": tuple(launch_xy), "person_xy": tuple(person_xy), "distance_m": distance,
            "yaw_rad": yaw, "person_ned": (dy, dx),
            "model_pose": "%.3f,%.3f,0,0,0,%.5f" % (launch_xy[0], launch_xy[1], yaw)}


def check_leg(north_m, east_m, speed_mps, alt_m):
    """Raise ValueError unless a fly-to-point leg is inside the limits."""
    if not all(math.isfinite(v) for v in (north_m, east_m, speed_mps, alt_m)):
        raise ValueError("leg values must be finite")
    if math.hypot(north_m, east_m) > MAX_LEG_M:
        raise ValueError("leg longer than %.0f m" % MAX_LEG_M)
    if not 0.0 < speed_mps <= MAX_SPEED_MPS:
        raise ValueError("speed must be in (0, %.0f] m/s" % MAX_SPEED_MPS)
    if not MIN_ALT_M <= alt_m <= MAX_ALT_M:
        raise ValueError("height must be in [%.1f, %.0f] m" % (MIN_ALT_M, MAX_ALT_M))


def leg_offset(pos_ne, person_ne, max_leg_m=MAX_LEG_M, stop_short_m=0.0):
    """(north, east, remaining_m): the next leg towards the person, at most max_leg_m long and
    never ending closer than stop_short_m to them. remaining_m is the full distance to them."""
    dn, de = person_ne[0] - pos_ne[0], person_ne[1] - pos_ne[1]
    remaining = math.hypot(dn, de)
    leg = min(max_leg_m, max(0.0, remaining - stop_short_m))
    if remaining == 0.0 or leg == 0.0:
        return 0.0, 0.0, remaining
    k = leg / remaining
    return dn * k, de * k, remaining


def follow_geometry_ok(height_m, follow_distance_m):
    """True when a person follow_distance_m ahead, 1.9 m tall, keeps their feet in frame from a
    camera height_m above the ground that looks level."""
    return follow_distance_m > 0 and height_m / follow_distance_m <= FEET_IN_FRAME


def movement_params_text(follow_distance_m=FOLLOW_DISTANCE_M):
    """Movement parameter file: follow at follow_distance_m inside its clamp band."""
    return ("/**:\n  ros__parameters:\n    safe_distance_m: %.1f\n    min_distance_m: %.1f\n"
            "    max_distance_m: %.1f\n" % (follow_distance_m, MOVEMENT_MIN_DISTANCE_M,
                                             MOVEMENT_MAX_DISTANCE_M))


def offset_latlon(lat_deg, lon_deg, north_m, east_m):
    """Latitude and longitude (deg) of the point north_m / east_m away (flat-earth)."""
    return (lat_deg + math.degrees(north_m / EARTH_RADIUS_M),
            lon_deg + math.degrees(east_m / (EARTH_RADIUS_M * math.cos(math.radians(lat_deg)))))


def range_ok(measured_m, true_m):
    """Tracking's range to the person against the known one."""
    return abs(measured_m - true_m) <= max(RANGE_TOL_MIN_M, RANGE_TOL_FRACTION * true_m)


def in_follow_band(distance_m):
    return abs(distance_m - FOLLOW_DISTANCE_M) <= FOLLOW_TOL_M


def px4_mode_name(custom_mode):
    """Human name of a PX4 HEARTBEAT custom_mode (Hold, Offboard, Return, ...)."""
    main = (int(custom_mode) >> 16) & 0xFF
    sub = (int(custom_mode) >> 24) & 0xFF
    if main == 4:
        return PX4_AUTO_MODES.get(sub, "Auto(%d)" % sub)
    return PX4_MAIN_MODES.get(main, "Unknown(%d)" % main)


def seen_enough(times, now, count=SEEN_COUNT, window_s=SEEN_WINDOW_S):
    """True when at least `count` detections arrived within the last `window_s` seconds."""
    return sum(1 for t in times if now - t <= window_s) >= count


# ================================================================================================
# The simulator and its services (processes, each in its own session)
# ================================================================================================

procs = {}


def say(message):
    print("[%s] pilot: %s" % (time.strftime("%H:%M:%S"), message), flush=True)


def start(name, cmd, env=None):
    os.makedirs(LOG_DIR, exist_ok=True)
    full_env = dict(os.environ)
    full_env.update(env or {})
    log = open(os.path.join(LOG_DIR, name + ".log"), "w")
    # stdin from /dev/null would make PX4's console spin; `fly` is started with -d (daemon) below
    procs[name] = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, env=full_env,
                                   start_new_session=True)
    return procs[name]


def stop(name, grace_s=8.0):
    proc = procs.pop(name, None)
    if proc is None:
        return
    if proc.poll() is None:
        try:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=grace_s)
        except (subprocess.TimeoutExpired, ProcessLookupError):
            pass
    deadline = time.monotonic() + grace_s       # a launcher can exit before its nodes do
    while time.monotonic() < deadline:
        try:
            os.killpg(proc.pid, 0)
        except ProcessLookupError:
            return
        time.sleep(0.2)
    try:
        os.killpg(proc.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass


def run(cmd, timeout=30.0):
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return out.returncode, (out.stdout + out.stderr).strip()
    except subprocess.TimeoutExpired:
        return 124, "timed out"


def leftovers():
    me = {os.getpid(), os.getppid()}
    rc, out = run(["pgrep", "-af", LEFTOVER], timeout=10.0)
    return [line for line in out.splitlines()
            if line and int(line.split()[0]) not in me and "baylands_mission" not in line
            and "pgrep" not in line]


def world_up():
    rc, out = run(["gz", "service", "-l"], timeout=10.0)
    return rc == 0 and ("/world/%s/create" % WORLD) in out.split()


def bring_up(geometry, gui):
    say("starting the simulator (%s) in %s, drone at x=%.1f y=%.1f"
        % ("with the Gazebo window" if gui else "headless", WORLD, *geometry["launch_xy"]))
    env = {"PX4_GZ_WORLD": WORLD, "PX4_GZ_MODEL_POSE": geometry["model_pose"],
           "HALO_SIM_WAIT_S": str(int(WORLD_WAIT_S))}
    # -d: PX4 daemon mode (no interactive console, which otherwise spins on a closed stdin)
    if gui:
        start("fly", ["vglrun", "-d", "egl", FLY, "-d"],
              dict(env, DISPLAY=os.environ.get("DISPLAY", ":0"), VGL_DISPLAY="egl"))
    else:
        start("fly", [FLY, "-d"], dict(env, HEADLESS="1"))
    deadline = time.monotonic() + WORLD_WAIT_S
    while time.monotonic() < deadline and not world_up():
        if procs["fly"].poll() is not None:
            raise RuntimeError("the simulator exited early, see %s/fly.log" % LOG_DIR)
        time.sleep(3.0)
    if not world_up():
        raise RuntimeError("no %s world after %.0f s, see %s/fly.log" % (WORLD, WORLD_WAIT_S,
                                                                          LOG_DIR))
    say("world is up")
    start("sim_links", [SIM_LINKS])
    start("geolocation", ["ros2", "launch", "geolocation", "geolocation.launch.py"])
    start("tracking", ["ros2", "launch", "tracking", "tracking.launch.py", "use_sim_time:=true"])
    start("perception", ["ros2", "launch", "perception", "perception.launch.py"]
          + (["overlay:=true"] if gui else []))
    start("ui", ["ros2", "launch", "ui", "ui.launch.py"])
    if gui:   # the camera as Perception sees it, with the detected box drawn on it
        start("perception_view", ["ros2", "run", "perception", "debug_view"],
              {"DISPLAY": os.environ.get("DISPLAY", ":0")})
    rc, out = run([SPAWN_PERSON, "%.3f" % geometry["person_xy"][0],
                   "%.3f" % geometry["person_xy"][1], "%.5f" % geometry["yaw_rad"]], timeout=60.0)
    say("person spawned at x=%.1f y=%.1f (rc=%s)" % (*geometry["person_xy"], rc))


def tear_down():
    say("shutting the simulator down")
    for name in ("movement", "perception_view", "ui", "tracking", "geolocation", "perception",
                 "sim_links", "fly"):
        stop(name)
    run(["ros2", "daemon", "stop"], timeout=15.0)
    time.sleep(2.0)
    left = leftovers()
    say("nothing left running" if not left else "LEFT RUNNING: " + "; ".join(l[:70] for l in left))
    return not left


# ================================================================================================
# The pilot: a MAVLink ground station (the local TCP bridge) plus the ROS topics it watches
# ================================================================================================

class Pilot:
    """Flies the vehicle like QGroundControl would: arm, takeoff, fly-to-point, hold, land.

    Single threaded: pump() sends the ground-station heartbeat PX4 needs, drains incoming
    MAVLink, and spins the ROS subscriptions, so every wait loop must call it.
    """

    def __init__(self):
        from pymavlink import mavutil
        self.mav = mavutil
        self.cmd = mavutil.mavlink
        self.link = None           # opened by connect(), once the simulator is up
        self.last_beat = 0.0
        self.armed, self.mode = False, "?"
        self.local = None          # (x north, y east, z down, vx, vy, vz)
        self.global_pos = None     # (lat deg, lon deg, alt AMSL m, relative alt m)
        self.acks = {}
        self.detections = []       # monotonic arrival times of confident detections
        self.last_detection = None
        self.geo = None            # (lat, lon) of the drone from Geolocation
        self.tracking = None       # (lat, lon, ex, ey, ez) of the person from Tracking
        self.tracking_at = 0.0     # monotonic arrival time of the last Tracking message
        self.node = None

    # -- ROS side -----------------------------------------------------------------------------
    def open_ros(self):
        import rclpy
        from geolocation_msgs.msg import GeolocationState
        from perception_msgs.msg import Detection
        from rclpy.qos import qos_profile_sensor_data
        from tracking_msgs.msg import TrackingState
        self.rclpy = rclpy
        rclpy.init()
        self.node = rclpy.create_node("baylands_mission")
        qos = qos_profile_sensor_data   # best effort: matches any publisher

        def on_detection(msg):
            if msg.confidence >= SEEN_CONFIDENCE:
                self.detections = (self.detections + [time.monotonic()])[-20:]
                self.last_detection = msg

        self.node.create_subscription(Detection, "/perception/detection", on_detection, qos)
        self.node.create_subscription(
            GeolocationState, "/geolocation/position",
            lambda m: setattr(self, "geo", (m.latitude_deg, m.longitude_deg)), qos)
        def on_tracking(msg):
            self.tracking = (msg.latitude_deg, msg.longitude_deg, msg.error_x_m, msg.error_y_m,
                             msg.error_z_m)
            self.tracking_at = time.monotonic()

        self.node.create_subscription(TrackingState, "/tracking/position", on_tracking, qos)

    def close(self):
        try:
            if self.node is not None:
                self.node.destroy_node()
                self.rclpy.shutdown()
        except Exception:
            pass

    # -- the main loop body -------------------------------------------------------------------
    def pump(self):
        now = time.monotonic()
        if now - self.last_beat >= HEARTBEAT_PERIOD_S:
            self.link.mav.heartbeat_send(self.cmd.MAV_TYPE_GCS, self.cmd.MAV_AUTOPILOT_INVALID,
                                         0, 0, 0)
            self.last_beat = now
        while True:
            msg = self.link.recv_match(blocking=False)
            if msg is None:
                break
            kind = msg.get_type()
            if kind == "LOCAL_POSITION_NED":
                self.local = (msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz)
            elif kind == "GLOBAL_POSITION_INT":
                self.global_pos = (msg.lat / 1e7, msg.lon / 1e7, msg.alt / 1000.0,
                                   msg.relative_alt / 1000.0)
            elif kind == "COMMAND_ACK":
                self.acks[msg.command] = msg.result
            elif kind == "HEARTBEAT" and msg.autopilot == self.cmd.MAV_AUTOPILOT_PX4:
                # only the autopilot's own heartbeat: relayed GCS heartbeats read "disarmed"
                self.armed = bool(msg.base_mode & self.cmd.MAV_MODE_FLAG_SAFETY_ARMED)
                self.mode = px4_mode_name(msg.custom_mode)
        if self.node is not None:
            self.rclpy.spin_once(self.node, timeout_sec=0.0)
        time.sleep(0.05)

    def wait_until(self, predicate, timeout_s):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.pump()
            if predicate():
                return True
        return False

    def connect(self):
        say("connecting to the autopilot on tcp:%s:%d" % ENDPOINT)
        self.link = self.mav.mavlink_connection("tcp:%s:%d" % ENDPOINT,
                                                source_system=GCS_SYSTEM_ID,
                                                source_component=GCS_COMPONENT_ID)
        if self.link.wait_heartbeat(timeout=60) is None:
            raise RuntimeError("no heartbeat from PX4 (is the simulator up?)")
        return self.wait_until(lambda: self.local and self.global_pos and self.mode != "?", 30.0)

    # -- state --------------------------------------------------------------------------------
    def ne(self):
        return (self.local[0], self.local[1])

    def height_m(self):
        return -self.local[2]

    def speed_mps(self):
        return math.hypot(self.local[3], self.local[4])

    # -- commands -----------------------------------------------------------------------------
    def command(self, command, *params, timeout_s=ACK_TIMEOUT_S):
        self.acks.pop(command, None)
        self.link.mav.command_long_send(self.link.target_system, self.link.target_component,
                                        command, 0, *(list(params) + [0] * (7 - len(params))))
        self.wait_until(lambda: command in self.acks, timeout_s)
        return self.acks.get(command) == self.cmd.MAV_RESULT_ACCEPTED

    def arm(self):
        """PX4 refuses until its sensors and position estimate are ready, so retry."""
        ok = self.wait_until(lambda: self.command(self.cmd.MAV_CMD_COMPONENT_ARM_DISARM, 1) and
                             self.wait_until(lambda: self.armed, 5.0), ARM_WAIT_S)
        return ok

    def takeoff(self, alt_m):
        if not MIN_ALT_M <= alt_m <= MAX_ALT_M:
            raise ValueError("takeoff height outside [%.1f, %.0f] m" % (MIN_ALT_M, MAX_ALT_M))
        # PX4 takes the height from MIS_TAKEOFF_ALT (above the takeoff point); the command's own
        # altitude is AMSL, so leave it NaN.
        self.link.param_set_send("MIS_TAKEOFF_ALT", alt_m)
        nan = float("nan")
        return self.command(self.cmd.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, nan, nan, nan, nan)

    def reposition(self, lat, lon, alt_amsl, speed_mps):
        """PX4's "go to location": fly there at speed_mps, then hold (MAV_CMD_DO_REPOSITION)."""
        command = self.cmd.MAV_CMD_DO_REPOSITION
        self.acks.pop(command, None)
        # COMMAND_INT keeps latitude and longitude as 1e7-scaled integers, so no float32 rounding.
        self.link.mav.command_int_send(
            self.link.target_system, self.link.target_component, self.cmd.MAV_FRAME_GLOBAL,
            command, 0, 0, speed_mps, self.cmd.MAV_DO_REPOSITION_FLAGS_CHANGE_MODE, 0,
            float("nan"), int(round(lat * 1e7)), int(round(lon * 1e7)), alt_amsl)
        self.wait_until(lambda: command in self.acks, ACK_TIMEOUT_S)
        return self.acks.get(command) == self.cmd.MAV_RESULT_ACCEPTED

    def fly_leg(self, north_m, east_m, speed_mps):
        """One bounded leg from where the vehicle is now, at the same height."""
        check_leg(north_m, east_m, speed_mps, self.height_m())
        lat, lon, alt, _ = self.global_pos
        lat, lon = offset_latlon(lat, lon, north_m, east_m)
        return self.reposition(lat, lon, alt, speed_mps)

    def hold(self):
        """Stop where it is: go to the current position."""
        if self.global_pos is None:
            return False
        lat, lon, alt, _ = self.global_pos
        return self.reposition(lat, lon, alt, -1.0)

    def land(self):
        ok = self.command(self.cmd.MAV_CMD_NAV_LAND)
        self.wait_until(lambda: self.local and self.local[2] > LANDED_Z_M, LAND_WAIT_S)
        self.command(self.cmd.MAV_CMD_COMPONENT_ARM_DISARM, 0)
        self.wait_until(lambda: not self.armed, 15.0)
        return ok and not self.armed


# ================================================================================================
# The mission
# ================================================================================================

def fly_approach(pilot, person_ne):
    """Fly towards the person in short legs until ENGAGE_DISTANCE_M away, noting when Perception
    first sees them. Returns (first_seen_m, flown_m, remaining_m, reason); reason is None when
    the engage point was reached. The vehicle is holding when this returns."""
    start = pilot.ne()
    started = time.monotonic()
    first_seen_m = None
    reason = None
    while reason is None:
        cur = pilot.ne()
        dn, de, remaining = leg_offset(cur, person_ne, MAX_LEG_M, ENGAGE_DISTANCE_M)
        if remaining - ENGAGE_DISTANCE_M <= ARRIVE_M:
            break                                  # close enough: this is the engage point
        if not pilot.fly_leg(dn, de, MAX_SPEED_MPS):
            reason = "the autopilot refused the fly-to-point command"
            break
        end = (cur[0] + dn, cur[1] + de)
        best, progress_at = math.hypot(dn, de), time.monotonic()
        while reason is None:
            pilot.pump()
            now = time.monotonic()
            if first_seen_m is None and seen_enough(pilot.detections, now):
                first_seen_m = math.dist(pilot.ne(), person_ne)
                say("person spotted %.0f m ahead (confidence %.2f); closing in"
                    % (first_seen_m, pilot.last_detection.confidence))
            left = math.dist(pilot.ne(), end)
            if left <= ARRIVE_M:
                break
            if left < best - 0.5:
                best, progress_at = left, now
            elif now - progress_at > STALL_S:
                reason = "no progress for %.0f s" % STALL_S
            elif now - started > APPROACH_TOTAL_S:
                reason = "the approach took more than %.0f s" % APPROACH_TOTAL_S
            elif math.dist(start, pilot.ne()) > MAX_FLOWN_M:
                reason = "flew more than %.0f m" % MAX_FLOWN_M
    pilot.hold()
    pilot.wait_until(lambda: pilot.speed_mps() < 0.5, STOP_WAIT_S)
    here = pilot.ne()
    return first_seen_m, math.dist(start, here), math.dist(here, person_ne), reason


def tracking_range_m(pilot):
    """Tracking's distance from the drone to the person (both are LLA), or None."""
    if not pilot.geo or not pilot.tracking:
        return None
    dn = math.radians(pilot.tracking[0] - pilot.geo[0]) * EARTH_RADIUS_M
    de = (math.radians(pilot.tracking[1] - pilot.geo[1]) * EARTH_RADIUS_M
          * math.cos(math.radians(pilot.geo[0])))
    return math.hypot(dn, de)


def mission(args, geometry):
    """Fly the whole mission. Returns the summary dict; raises nothing the caller cannot handle."""
    summary = {"seen": False, "followed": False, "landed": False}
    person_ne = geometry["person_ned"]
    pilot = Pilot()
    try:
        bring_up(geometry, not args.headless)
        pilot.open_ros()
        if not pilot.connect():
            raise RuntimeError("no position from the autopilot")
        say("pre-flight: waiting for the position estimate, then arming")
        if not pilot.arm():
            raise RuntimeError("could not arm within %.0f s (see %s/fly.log)"
                               % (ARM_WAIT_S, LOG_DIR))
        say("armed, climbing to %.1f m (20 ft)" % CRUISE_ALT_M)
        if not pilot.takeoff(CRUISE_ALT_M):
            raise RuntimeError("takeoff was refused")
        if not pilot.wait_until(lambda: pilot.height_m() >= 0.9 * CRUISE_ALT_M, CLIMB_WAIT_S):
            raise RuntimeError("did not reach %.1f m (at %.1f m)" % (CRUISE_ALT_M,
                                                                       pilot.height_m()))
        say("at %.1f m. Flying towards the person, %.0f m away, in legs of %.0f m at %.0f m/s, "
            "watching the camera" % (pilot.height_m(), geometry["distance_m"], MAX_LEG_M,
                                     MAX_SPEED_MPS))
        first_seen_m, flown, remaining, reason = fly_approach(pilot, person_ne)
        summary.update(flown_m=flown, remaining_m=remaining, first_seen_m=first_seen_m)
        if reason:
            raise RuntimeError("%s (flew %.0f m, %.0f m from the person)"
                               % (reason, flown, remaining))
        say("at the engage point: %.0f m from the person after flying %.0f m, holding at %.1f m"
            % (remaining, flown, pilot.height_m()))

        say("checking that Perception and Tracking both have them")
        if not pilot.wait_until(lambda: seen_enough(pilot.detections, time.monotonic()),
                                REDETECT_WAIT_S):
            raise RuntimeError("Perception does not see the person from the engage point")
        summary["seen"] = True
        if not pilot.wait_until(lambda: time.monotonic() - pilot.tracking_at <= TRACKING_FRESH_S,
                                TRACKING_WAIT_S):
            raise RuntimeError("Tracking published no position for the person")
        measured = tracking_range_m(pilot)
        true_m = math.dist(pilot.ne(), person_ne)
        summary.update(tracking_m=measured, true_m=true_m)
        say("Tracking says %.1f m, the map says %.1f m: %s" % (
            measured, true_m, "agreed" if range_ok(measured, true_m) else "DISAGREE"))
        if not range_ok(measured, true_m):
            raise RuntimeError("Tracking's range is off by more than the tolerance")
        if not follow_geometry_ok(pilot.height_m(), FOLLOW_DISTANCE_M):
            raise RuntimeError("at %.1f m the person's feet leave the frame inside %.0f m"
                               % (pilot.height_m(), FOLLOW_DISTANCE_M))

        say("switching on the automatic follow (Movement), asked to follow at %.0f m"
            % FOLLOW_DISTANCE_M)
        os.makedirs(LOG_DIR, exist_ok=True)
        with open(MOVEMENT_PARAMS, "w") as fh:
            fh.write(movement_params_text())
        start("movement", ["ros2", "launch", "movement", "movement.launch.py",
                           "params_file:=" + MOVEMENT_PARAMS])
        movement_started = time.monotonic()
        band_since = None
        end = time.monotonic() + FOLLOW_TIMEOUT_S
        closest = float("inf")
        while time.monotonic() < end:
            pilot.pump()
            if procs["movement"].poll() is not None:
                raise RuntimeError("Movement exited, see %s/movement.log" % LOG_DIR)
            if (time.monotonic() - max(pilot.tracking_at, movement_started) > LOST_ABORT_S):
                raise RuntimeError("Tracking lost the person for %.0f s while Movement was on"
                                   % LOST_ABORT_S)
            dist = math.dist(pilot.ne(), person_ne)
            closest = min(closest, dist)
            if in_follow_band(dist):
                band_since = band_since or time.monotonic()
                if time.monotonic() - band_since >= FOLLOW_STABLE_S:
                    summary.update(followed=True, follow_m=dist)
                    break
            else:
                band_since = None
        say("following at %.1f m from the person (closest %.1f m)"
            % (math.dist(pilot.ne(), person_ne), closest))
        if not summary["followed"]:
            raise RuntimeError("Movement never settled within %.1f m of %.0f m"
                               % (FOLLOW_TOL_M, FOLLOW_DISTANCE_M))
        stop("movement")
        got_hold = pilot.wait_until(lambda: pilot.mode == "Hold", STOP_WAIT_S)
        say("Movement off, PX4 mode %s" % pilot.mode)
        summary["handed_back_to_hold"] = got_hold
        return summary, pilot
    except KeyboardInterrupt:
        say("interrupted")
        summary["error"] = "interrupted"
    except Exception as err:   # noqa: BLE001 - anything: land and report
        say("ABORT: %s" % err)
        summary["error"] = str(err)
    return summary, pilot


def finish(args, pilot, summary):
    """Whatever happened: stop moving, put it on the ground, shut the sim down."""
    try:
        stop("movement")
        if pilot.link is not None and pilot.local is not None:
            pilot.pump()
            if pilot.armed:
                say("holding, then landing")
                pilot.hold()
                summary["landed"] = pilot.land()
                say("landed and disarmed" if summary["landed"] else "LANDING NOT CONFIRMED")
            else:
                summary["landed"] = True
        if args.linger > 0 and not args.headless:
            say("lingering %.0f s with the window up" % args.linger)
            deadline = time.monotonic() + args.linger
            while time.monotonic() < deadline:
                if pilot.link is not None:
                    pilot.pump()    # keeps the ground-station heartbeat going
                else:
                    time.sleep(0.5)
    finally:
        pilot.close()
        summary["clean_shutdown"] = tear_down()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--headless", action="store_true", help="no Gazebo window or Perception view")
    ap.add_argument("--linger", type=float, default=20.0, metavar="S",
                    help="seconds to keep the window up after landing (default %(default)s)")
    ap.add_argument("--plan", action="store_true", help="print the geometry and exit")
    ap.add_argument("--launch-xy", default=None, metavar="X,Y",
                    help="drone start, world metres (default %g,%g); use = for negatives"
                    % LAUNCH_XY)
    ap.add_argument("--person-xy", default=None, metavar="X,Y",
                    help="person position, world metres (default %g,%g)" % PERSON_XY)
    args = ap.parse_args(argv)
    try:
        geometry = plan(parse_xy(args.launch_xy) if args.launch_xy else LAUNCH_XY,
                        parse_xy(args.person_xy) if args.person_xy else PERSON_XY)
    except ValueError as err:
        print("error: %s" % err, file=sys.stderr)
        return 2
    print("launch x=%.2f y=%.2f  person x=%.2f y=%.2f  distance %.1f m  heading %.1f deg  "
          "person at north %.1f east %.1f from the launch point"
          % (*geometry["launch_xy"], *geometry["person_xy"], geometry["distance_m"],
             math.degrees(geometry["yaw_rad"]), *geometry["person_ned"]))
    if args.plan:
        return 0
    if not os.environ.get("PX4_SIM_MODEL"):
        print("error: PX4_SIM_MODEL is unset, so there is no simulator to fly", file=sys.stderr)
        return 2
    if not os.environ.get("HALO_TRITON_URL"):
        print("error: the mission needs Perception (HALO_TRITON_URL, Spark only)", file=sys.stderr)
        return 2
    if world_up() or leftovers():
        print("error: a simulator is already running; stop it first", file=sys.stderr)
        return 2
    started = time.monotonic()

    def interrupted(signum, _frame):
        raise KeyboardInterrupt()      # same path as Ctrl+C: hold, land, tear down

    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGHUP, interrupted)
    summary, pilot = mission(args, geometry)
    finish(args, pilot, summary)
    ok = (summary.get("followed") and summary.get("landed") and summary.get("clean_shutdown")
          and not summary.get("error"))
    fields = ", ".join("%s=%s" % (k, ("%.1f" % v) if isinstance(v, float) else v)
                       for k, v in summary.items())
    print("\nMISSION %s in %.0f s: %s" % ("COMPLETE" if ok else "FAILED",
                                         time.monotonic() - started, fields))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
