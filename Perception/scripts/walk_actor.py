#!/usr/bin/python3
# System python on purpose, not `env python3`: the gz-transport bindings,
# apt's python3-protobuf and python3-tk live in system dist-packages, which an
# activated virtualenv (e.g. GroundstationAPI/.venv) can't see.
"""Spawn an animated walking person in the running Gazebo world and drive
him from the keyboard.

He's a Gazebo actor (Perception/models/walking_person.sdf) running Gazebo's
built-in FollowActor system, which walks him toward an invisible target
(walk_target.sdf) with the walk animation playing in step with his feet.
This script spawns both, then moves the target from your keys ~30 times a
second, always a little ahead of where he should be. He stops when you let
go. Replaces standing_person / red_sphere if they're in the world, like
spawn_person.sh. Gazebo can't remove actors, so re-running it reuses him:
he walks over to the start point instead of reappearing there.

    walk_actor.py [x] [y] [yaw]    (defaults: 5 0 0, same as spawn_person:
                                    5m out, facing the drone at origin)

Opens a small window that reads keys. Keep it focused while you drive. A
window, not the terminal, because a terminal only reports the most recently
repeated key and never key releases, so you couldn't walk and turn at once.

Keys (combine freely, e.g. W+A walks forward while turning left):
    W / S     walk forward / back (back = he turns round and walks)
    A / D     turn left / right (alone: he shuffles round on the spot)
    Q / E     sidestep left / right (he turns that way and walks)
    X / Esc   quit (he stays in the world)

Needs python3-tk (and python3-protobuf for smooth updates). Set WORLD to
override the world name, same as the spawn scripts.
"""
import math
import os
import subprocess
import re
import sys
import time
import tkinter as tk

ACTOR = "walking_person"
TARGET = "walk_target"
MODELS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "models")
# Must match <velocity> in walking_person.sdf: he follows at that fixed speed,
# so a faster target would pull away and a slower one would make him stutter.
WALK_MPS = 1.4
TURN_RPS = math.radians(90)
# The target sits this far ahead of his intended position along his heading,
# so he keeps facing that way when he stops and can turn on the spot (he only
# ever faces where he's walking). Just above <min_distance> (0.02): he then
# stops ~2mm from the intended spot, so turning on the spot swings the target
# around him rather than past him and he ends up facing the right way.
LEAD_M = 0.022
TICK_MS = 33
# X11 autorepeat sends a fake release+press pair while a key is held; a
# release only counts if no press follows within this long.
RELEASE_DEBOUNCE_MS = 40


def find_world():
    if os.environ.get("WORLD"):
        return os.environ["WORLD"]
    out = subprocess.run(["gz", "service", "-l"], capture_output=True, text=True).stdout
    for line in out.splitlines():
        if line.startswith("/world/") and line.endswith("/create"):
            return line.split("/")[2]
    sys.exit("No running Gazebo world found — is `fly` running?")


def heading(yaw):
    """Unit forward vector. Same yaw convention as spawn_person.sh: yaw 0
    faces world -x, i.e. back toward a drone at origin."""
    return -math.cos(yaw), -math.sin(yaw)


def gz_service(world, name, reqtype, req, timeout_ms):
    return subprocess.run(
        ["gz", "service", "-s", f"/world/{world}/{name}", "--reqtype", reqtype,
         "--reptype", "gz.msgs.Boolean", "--timeout", str(timeout_ms), "--req", req],
        capture_output=True, text=True).stdout


def spawn(world, x, y):
    """Spawn target + actor, or reuse them if this script already did.

    Gazebo's remove service refuses actors ("not a model or a light"), and
    removing walk_target while the actor follows it segfaults the whole
    server (FollowActor keeps using the deleted entity). So neither is ever
    removed: if they exist (seen via the target; actors aren't listed in
    scene/info), the target just moves to the start point and he walks there.
    """
    scene = subprocess.run(
        ["gz", "service", "-s", f"/world/{world}/scene/info", "--reqtype", "gz.msgs.Empty",
         "--reptype", "gz.msgs.Scene", "--timeout", "3000", "--req", ""],
        capture_output=True, text=True).stdout
    if f'name: "{TARGET}"' in scene:
        print("Already spawned (actors can't be removed): he'll walk to the start point. "
              "Restart the sim for a fresh one.")
        return False

    for name in ("red_sphere", "standing_person"):
        gz_service(world, "remove", "gz.msgs.Entity", f'name: "{name}", type: MODEL', 2000)
    # Blocking: FollowActor looks its target up when the actor loads, so the
    # target must really exist before the actor is created. Placed exactly on
    # him: FollowActor's first update uses the whole sim time so far as its
    # time step, and would fling him past any target that isn't already
    # within min_distance. main() moves it ahead of him after that.
    gz_service(world, "create/blocking", "gz.msgs.EntityFactory",
               f'sdf_filename: "{os.path.join(MODELS, "walk_target.sdf")}", name: "{TARGET}", '
               f"pose: {{position: {{x: {x}, y: {y}, z: 0}}}}", 2000)
    # Start position in an <actor><pose>, not the request's pose: FollowActor
    # reads it on load, before a request pose would be applied, and would
    # walk him offset by the spawn point. z 1.0: actor origin is the torso.
    # Yaw 0 always (FollowActor would apply any spawn rotation twice); main()
    # then puts the target LEAD_M along `yaw` to turn him with one tiny step.
    with open(os.path.join(MODELS, "walking_person.sdf")) as f:
        sdf = f.read().replace("<skin>", f"<pose>{x} {y} 1.0 0 0 0</pose>\n    <skin>", 1)
    sdf = re.sub(r"<!--.*?-->", "", sdf, flags=re.S)  # just to keep the request small
    sdf = sdf.replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n")
    # create/blocking: plain create answers true before it even reads the
    # request, so it can't tell us whether the spawn worked. Long timeout:
    # the first spawn downloads the mesh from Gazebo Fuel.
    ok = gz_service(world, "create/blocking", "gz.msgs.EntityFactory",
                    f'sdf: "{sdf}", name: "{ACTOR}"', 30000)
    if "true" not in ok:
        sys.exit("Spawning the walking person failed (no internet for the first Fuel download?).")
    return True


class PoseSender:
    """Sends set_pose requests for the target without flooding the machine.

    Each `gz service` CLI call is a whole new process that takes a few hundred
    ms to start, so starting one per update piles up processes that starve
    Gazebo of CPU. Preferred path: the gz-transport Python bindings, one
    persistent node, no process per call. Fallback (bindings or protobuf
    missing): at most one `gz service` process in flight, always carrying the
    newest pose; poses in between are dropped.
    """

    def __init__(self, world):
        self.world = world
        self.service = f"/world/{world}/set_pose"
        self.pending = None
        self.proc = None
        try:
            from gz.transport13 import Node
            from gz.msgs10.pose_pb2 import Pose
            from gz.msgs10.boolean_pb2 import Boolean
            self.node, self.Pose, self.Boolean = Node(), Pose, Boolean
            self.mode = "gz-transport bindings"
        except ImportError as e:
            self.node = None
            self.mode = "gz CLI fallback (choppy)"
            print(f"gz-transport bindings unavailable ({e}); falling back to "
                  "the gz CLI, which updates less smoothly. "
                  "`apt install python3-protobuf` and run with /usr/bin/python3 "
                  "(not a virtualenv) to fix.")

    def send(self, x, y):
        if self.node:
            msg = self.Pose()
            msg.name = TARGET
            msg.position.x, msg.position.y = x, y
            msg.orientation.w = 1
            self.node.request(self.service, msg, self.Pose, self.Boolean, 200)
        else:
            self.pending = (x, y)
            self.flush()

    def flush(self):
        """Fallback path: start the next CLI call once the previous one exits."""
        if self.pending is None or (self.proc and self.proc.poll() is None):
            return
        x, y = self.pending
        self.pending = None
        self.proc = subprocess.Popen(
            ["gz", "service", "-s", self.service,
             "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean", "--timeout", "500",
             "--req", f'name: "{TARGET}", position: {{x: {x}, y: {y}, z: 0}}, orientation: {{w: 1}}'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )


class Walker:
    """Integrates the keys into where he should be (x, y, yaw) and puts the
    target LEAD_M ahead of that; FollowActor does the walking."""

    def __init__(self, root, sender, x, y, yaw):
        self.root, self.sender = root, sender
        self.x, self.y, self.yaw = x, y, yaw
        self.held = set()
        self.release_timers = {}
        self.last = time.monotonic()

        root.title("Walk actor")
        root.geometry("420x210")
        controls = __doc__.split("Keys")[1].split("Needs python3-tk")[0]
        controls = controls.split("\n", 1)[1].rstrip()
        tk.Label(root, text="Keep this window focused.\n" + controls,
                 justify="left", font=("monospace", 10)).pack(padx=10, pady=8, anchor="w")
        self.status = tk.Label(root, font=("monospace", 10), justify="left")
        self.status.pack(padx=10, anchor="w")

        root.bind("<KeyPress>", self.on_press)
        root.bind("<KeyRelease>", self.on_release)
        # Keys released while unfocused never send a release; drop them all
        # so he doesn't keep walking on his own.
        root.bind("<FocusOut>", lambda _e: self.held.clear())
        self.show()
        root.after(TICK_MS, self.tick)

    def on_press(self, event):
        k = event.keysym.lower()
        if k in ("x", "escape"):
            self.root.destroy()
            return
        timer = self.release_timers.pop(k, None)
        if timer:
            self.root.after_cancel(timer)  # autorepeat: key never actually went up
        self.held.add(k)

    def on_release(self, event):
        k = event.keysym.lower()
        self.release_timers[k] = self.root.after(
            RELEASE_DEBOUNCE_MS, lambda: (self.held.discard(k), self.release_timers.pop(k, None)))

    def tick(self):
        now = time.monotonic()
        dt, self.last = now - self.last, now
        h = self.held
        fwd = ("w" in h or "up" in h) - ("s" in h or "down" in h)
        turn = ("a" in h or "left" in h) - ("d" in h or "right" in h)
        side = ("q" in h) - ("e" in h)

        if fwd or turn or side:
            self.yaw += turn * TURN_RPS * dt
            fx, fy = heading(self.yaw)
            lx, ly = -fy, fx  # left = forward rotated +90deg
            # Normalize so diagonal (forward + sidestep) isn't faster than he walks.
            step = WALK_MPS * dt / (math.hypot(fwd, side) or 1)
            self.x += (fwd * fx + side * lx) * step
            self.y += (fwd * fy + side * ly) * step
            self.sender.send(self.x + fx * LEAD_M, self.y + fy * LEAD_M)
            self.show()
        else:
            self.sender.flush()
        self.root.after(TICK_MS, self.tick)

    def show(self):
        self.status.config(text=(
            f"x={self.x:7.2f}  y={self.y:7.2f}  yaw={math.degrees(self.yaw):6.1f}°\n"
            f"via {self.sender.mode}"))


def main():
    x = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
    y = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    yaw = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    world = find_world()
    print(f"Spawning '{ACTOR}' in world '{world}' at ({x}, {y})...")
    if spawn(world, x, y):
        time.sleep(0.5)  # let FollowActor's first update pass (see spawn())
    print("Drive him from the 'Walk actor' window.")
    root = tk.Tk()
    sender = PoseSender(world)
    # Face him along `yaw` (fresh spawn), or walk him to the start (reused).
    fx, fy = heading(yaw)
    sender.send(x + fx * LEAD_M, y + fy * LEAD_M)
    Walker(root, sender, x, y, yaw)
    root.mainloop()


if __name__ == "__main__":
    main()
