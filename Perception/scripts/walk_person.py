#!/usr/bin/python3
# System python on purpose, not `env python3`: the gz-transport bindings,
# apt's python3-protobuf and python3-tk live in system dist-packages, which an
# activated virtualenv (e.g. GroundstationAPI/.venv) can't see.
"""Keyboard-drive the standing_person model around the running Gazebo world.

Moves the person spawn_person / perceive already put in the world (the
Gazebo Fuel "Standing person" mesh, Perception/models/standing_person.sdf).
That mesh is a single frozen pose with no skeleton or animations, so he
glides rather than walks. For an animated (but different-looking) walker,
see walk_actor.py.

Opens a small window that reads keys. Keep it focused while you drive. A
window, not the terminal, because a terminal only reports the most recently
repeated key and never key releases, so you couldn't walk and turn at once.
Here every key is tracked as held or not, so any combination works.

The person is a static model, so instead of physics this moves him with the
world's set_pose service ~30 times a second while keys are held.

Spawn him first (spawn_person [x] [y] [yaw], or perceive), then run this
with the same x y yaw so it starts from where he actually is:

    walk_person.py [x] [y] [yaw]        (defaults: 5 0 0, same as spawn_person)

Keys (combine freely, e.g. W+A walks forward while turning left):
    W / S     walk forward / backward
    A / D     turn left / right
    Q / E     sidestep left / right
    Shift     run
    X / Esc   quit

Needs python3-tk (and python3-protobuf for smooth updates). Set WORLD to
override the world name, same as the spawn scripts.
"""
import math
import os
import subprocess
import sys
import time
import tkinter as tk

MODEL = "standing_person"
WALK_MPS = 1.4
RUN_MPS = 4.0
TURN_RPS = math.radians(90)
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


class PoseSender:
    """Sends set_pose requests without flooding the machine.

    Each `gz service` CLI call is a whole new process that takes a few hundred
    ms to start, so starting one per update piles up processes that starve
    Gazebo of CPU. Preferred path: the gz-transport Python bindings, one
    persistent node, no process per call. Fallback (bindings or protobuf
    missing): at most one `gz service` process in flight, always carrying the
    newest pose; poses in between are dropped.
    """

    def __init__(self, world):
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

    def send(self, x, y, yaw):
        if self.node:
            msg = self.Pose()
            msg.name = MODEL
            msg.position.x, msg.position.y = x, y
            msg.orientation.z, msg.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            self.node.request(self.service, msg, self.Pose, self.Boolean, 200)
        else:
            self.pending = (x, y, yaw)
            self.flush()

    def flush(self):
        """Fallback path: start the next CLI call once the previous one exits."""
        if self.pending is None or (self.proc and self.proc.poll() is None):
            return
        x, y, yaw = self.pending
        self.pending = None
        req = (
            f'name: "{MODEL}", position: {{x: {x}, y: {y}, z: 0}}, '
            f"orientation: {{z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}"
        )
        self.proc = subprocess.Popen(
            ["gz", "service", "-s", self.service,
             "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
             "--timeout", "500", "--req", req],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )


class Walker:
    def __init__(self, root, sender, x, y, yaw):
        self.root, self.sender = root, sender
        self.x, self.y, self.yaw = x, y, yaw
        self.held = set()
        self.release_timers = {}
        self.last = time.monotonic()

        root.title("Walk person")
        root.geometry("340x230")
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

    @staticmethod
    def key_name(event):
        k = event.keysym.lower()
        return "shift" if k.startswith("shift") else k

    def on_press(self, event):
        k = self.key_name(event)
        if k in ("x", "escape"):
            self.root.destroy()
            return
        timer = self.release_timers.pop(k, None)
        if timer:
            self.root.after_cancel(timer)  # autorepeat: key never actually went up
        self.held.add(k)

    def on_release(self, event):
        k = self.key_name(event)
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
            speed = RUN_MPS if "shift" in h else WALK_MPS
            self.yaw += turn * TURN_RPS * dt
            # The model faces world -x at yaw 0 (see standing_person.sdf),
            # so "forward" is (-cos yaw, -sin yaw) and "left" is its +90deg.
            fx, fy = -math.cos(self.yaw), -math.sin(self.yaw)
            lx, ly = -fy, fx
            # Normalize so diagonal (forward + sidestep) isn't faster.
            step = speed * dt / (math.hypot(fwd, side) or 1)
            self.x += (fwd * fx + side * lx) * step
            self.y += (fwd * fy + side * ly) * step
            self.sender.send(self.x, self.y, self.yaw)
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
    print(f"Driving '{MODEL}' in world '{world}'. Use the 'Walk person' window.")
    root = tk.Tk()
    Walker(root, PoseSender(world), x, y, yaw)
    root.mainloop()


if __name__ == "__main__":
    main()
