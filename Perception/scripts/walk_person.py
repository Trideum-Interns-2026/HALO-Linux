#!/usr/bin/python3
# System python on purpose, not `env python3`: the gz-transport bindings,
# apt's python3-protobuf and python3-tk live in system dist-packages, which an
# activated virtualenv (e.g. GroundstationAPI/.venv) can't see.
"""Spawn the animated person in the running Gazebo world and drive him from
the keyboard: walking, sprinting and dancing, with matching animations.

He's the project's standing person (same mesh and textures) rigged and
animated on Mixamo, converted by prepare_actor.py into
Perception/models/animated_person/. Gazebo can only animate an <actor>, and
has no built-in way to steer one live, so he carries a small plugin
(Perception/gz_plugins/actor_pose_control, built automatically on first
run) that applies what this script sends: position, facing, animation and
how far into it. The script moves him and picks the animation; walk/run
advance by distance walked at each animation's natural speed, so his feet
don't slide.

    walk_person.py [x] [y] [yaw]   (defaults: 5 0 0, same as spawn_person:
                                    5m out, facing the drone at origin)

Replaces standing_person / red_sphere if they're in the world. Gazebo can't
remove actors, so if he's already there (from an earlier run) he's reused
from where he stands and x/y/yaw are ignored; restart the sim for a fresh
one.

Opens a small window that reads keys. Keep it focused while you drive.

Keys (combine freely, e.g. W+A walks forward while turning left):
    W / S       walk forward / backward
    A / D       turn left / right
    Q / E       sidestep left / right
    Shift       sprint (run animation)
    Space       dance (toggle; moving stops it)
    X / Esc     quit (he stays in the world)

Needs python3-tk and python3-protobuf. Set WORLD to override the world name,
same as the spawn scripts.
"""
import bisect
import json
import math
import os
import subprocess
import sys
import time
import tkinter as tk

ACTOR = "animated_person"
HERE = os.path.dirname(os.path.abspath(__file__))
MODEL_DIR = os.path.normpath(os.path.join(HERE, "..", "models", "animated_person"))
PLUGIN_DIR = os.path.normpath(os.path.join(HERE, "..", "gz_plugins", "actor_pose_control"))
PLUGIN_SO = os.path.join(PLUGIN_DIR, "build", "libActorPoseControl.so")
TURN_RPS = math.radians(120)
TURN_STEP_RATE = 0.5    # walk animation speed while turning on the spot
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


def ensure_plugin():
    """Build the actor plugin if it's missing or older than its source."""
    srcs = [os.path.join(PLUGIN_DIR, f) for f in ("ActorPoseControl.cc", "CMakeLists.txt")]
    if os.path.exists(PLUGIN_SO) and os.path.getmtime(PLUGIN_SO) > max(map(os.path.getmtime, srcs)):
        return
    print("Building the actor plugin (first run only, ~30s)...")
    build = os.path.join(PLUGIN_DIR, "build")
    for cmd in (["cmake", "-S", PLUGIN_DIR, "-B", build, "-DCMAKE_BUILD_TYPE=Release"],
                ["cmake", "--build", build, "-j4"]):
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode:
            sys.exit(f"Building the actor plugin failed:\n{r.stdout[-2000:]}{r.stderr[-2000:]}")


def load_meta():
    path = os.path.join(MODEL_DIR, "meta.json")
    if not os.path.exists(path):
        sys.exit(f"{path} missing: run Perception/scripts/prepare_actor.py first.")
    with open(path) as f:
        meta = json.load(f)
    if "walk" not in meta["animations"]:
        sys.exit("The animated person has no walk animation; re-run prepare_actor.py.")
    return meta


def actor_sdf(meta, x, y):
    anims = "\n".join(
        f'    <animation name="{name}"><filename>{os.path.join(MODEL_DIR, a["file"])}</filename></animation>'
        for name, a in meta["animations"].items())
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <actor name="{ACTOR}">
    <pose>{x} {y} {meta.get("root_height_m", 1.0)} 0 0 0</pose>
    <skin><filename>{os.path.join(MODEL_DIR, meta["skin"])}</filename></skin>
{anims}
    <plugin filename="{PLUGIN_SO}" name="ActorPoseControl"/>
  </actor>
</sdf>"""


def gz_service(world, name, reqtype, req, timeout_ms):
    return subprocess.run(
        ["gz", "service", "-s", f"/world/{world}/{name}", "--reqtype", reqtype,
         "--reptype", "gz.msgs.Boolean", "--timeout", str(timeout_ms), "--req", req],
        capture_output=True, text=True).stdout


def spawn(world, meta, x, y):
    for name in ("red_sphere", "standing_person"):
        gz_service(world, "remove", "gz.msgs.Entity", f'name: "{name}", type: MODEL', 2000)
    sdf = actor_sdf(meta, x, y).replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n")
    # create/blocking: plain create answers true before it even reads the
    # request, so it can't tell us whether the spawn worked.
    ok = gz_service(world, "create/blocking", "gz.msgs.EntityFactory", f'sdf: "{sdf}"', 30000)
    if "true" not in ok:
        sys.exit("Spawning the animated person failed; see the Gazebo terminal for why.")


class Link:
    """Talks to the actor's plugin: /actor/<name>/cmd out, /actor/<name>/state in."""

    def __init__(self):
        self.cmd_topic = f"/actor/{ACTOR}/cmd"
        self.state_topic = f"/actor/{ACTOR}/state"
        self.proc = None
        self.pending = None
        self.state = None
        try:
            from gz.transport13 import Node
            from gz.msgs10.stringmsg_pb2 import StringMsg
            self.StringMsg = StringMsg
            self.node = Node()
            self.pub = self.node.advertise(self.cmd_topic, StringMsg)
            self.node.subscribe(StringMsg, self.state_topic, self._on_state)
            self.mode = "gz-transport bindings"
        except ImportError as e:
            self.node = None
            self.mode = "gz CLI fallback (choppy)"
            print(f"gz-transport bindings unavailable ({e}); falling back to the gz CLI, "
                  "which updates less smoothly. `apt install python3-protobuf` and run "
                  "with /usr/bin/python3 (not a virtualenv) to fix.")

    def _on_state(self, msg):
        self.state = msg.data

    def existing(self, wait_s=1.5):
        """(x, y, yaw) if the actor is already in the world, else None."""
        if self.node:
            end = time.time() + wait_s
            while self.state is None and time.time() < end:
                time.sleep(0.05)
            data = self.state
        else:
            out = subprocess.run(["timeout", str(wait_s + 1), "gz", "topic", "-e", "-n", "1",
                                  "-t", self.state_topic], capture_output=True, text=True).stdout
            data = out.split('"')[1] if '"' in out else None
        if not data:
            return None
        x, y, _z, yaw = (float(v) for v in data.split()[:4])
        return x, y, yaw

    def send(self, x, y, z, yaw, q, anim, t):
        """q: hips orientation [w, x, y, z] (facing + the clip's rotation)."""
        data = (f"{x:.4f} {y:.4f} {z:.4f} {yaw:.4f} "
                + " ".join(f"{v:.5f}" for v in q) + f" {anim} {t:.4f}")
        if self.node:
            msg = self.StringMsg()
            msg.data = data
            self.pub.publish(msg)
        else:
            self.pending = data
            self.flush()

    def flush(self):
        """CLI fallback: one `gz topic` process at a time, newest command only."""
        if self.node or self.pending is None or (self.proc and self.proc.poll() is None):
            return
        data, self.pending = self.pending, None
        self.proc = subprocess.Popen(
            ["gz", "topic", "-t", self.cmd_topic, "-m", "gz.msgs.StringMsg", "-p", f'data: "{data}"'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


class Walker:
    def __init__(self, root, link, meta, x, y, face):
        self.root, self.link = root, link
        self.anims = meta["animations"]
        self.root_height = meta.get("root_height_m", 1.0)
        self.x, self.y, self.face = x, y, face   # face: 0 = looking along +X
        self.anim, self.t = "walk", 0.0
        self.dancing = False
        self.held = set()
        self.release_timers = {}
        self.last = time.monotonic()
        self.walk_mps = self.anims["walk"]["ground_speed_mps"] or 1.4
        run = self.anims.get("run")
        self.run_mps = (run or {}).get("ground_speed_mps") or 2 * self.walk_mps

        root.title("Walk person")
        root.geometry("460x250")
        controls = __doc__.split("Keys")[1].split("Needs python3-tk")[0]
        controls = controls.split("\n", 1)[1].rstrip()
        missing = [a for a in ("run", "dance") if a not in self.anims]
        note = f"\n(no {' or '.join(missing)} animation yet: add it and re-run prepare_actor.py)" if missing else ""
        tk.Label(root, text="Keep this window focused.\n" + controls + note,
                 justify="left", font=("monospace", 10)).pack(padx=10, pady=8, anchor="w")
        self.status = tk.Label(root, font=("monospace", 10), justify="left")
        self.status.pack(padx=10, anchor="w")

        root.bind("<KeyPress>", self.on_press)
        root.bind("<KeyRelease>", self.on_release)
        # Keys released while unfocused never send a release; drop them all
        # so he doesn't keep walking on his own.
        root.bind("<FocusOut>", lambda _e: self.held.clear())
        self.send()
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
        elif k == "space" and "dance" in self.anims:
            self.dancing = not self.dancing
            if self.dancing:
                self.set_anim("dance", keep_phase=False)
        self.held.add(k)

    def on_release(self, event):
        k = self.key_name(event)
        self.release_timers[k] = self.root.after(
            RELEASE_DEBOUNCE_MS, lambda: (self.held.discard(k), self.release_timers.pop(k, None)))

    def set_anim(self, name, keep_phase=True):
        """Switch animation; walk<->run keep their place in the stride."""
        if name == self.anim:
            return
        old_len = self.anims[self.anim]["length_s"] or 1
        new_len = self.anims[name]["length_s"] or 1
        self.t = (self.t / old_len) * new_len if keep_phase else 0.0
        self.anim = name

    def tick(self):
        now = time.monotonic()
        dt, self.last = now - self.last, now
        h = self.held
        fwd = ("w" in h or "up" in h) - ("s" in h or "down" in h)
        turn = ("a" in h or "left" in h) - ("d" in h or "right" in h)
        side = ("q" in h) - ("e" in h)
        sprint = "shift" in h
        changed = True

        if fwd or side:
            self.dancing = False
            running = sprint and "run" in self.anims
            self.set_anim("run" if running else "walk")
            speed = self.run_mps if sprint else self.walk_mps
            anim_speed = self.anims[self.anim]["ground_speed_mps"] or speed
            self.face += turn * TURN_RPS * dt
            fx, fy = math.cos(self.face), math.sin(self.face)
            lx, ly = -fy, fx  # left = forward rotated +90deg
            # Normalize so diagonal (forward + sidestep) isn't faster.
            step = speed * dt / math.hypot(fwd, side)
            self.x += (fwd * fx + side * lx) * step
            self.y += (fwd * fy + side * ly) * step
            # Advance the cycle by distance at the animation's own speed (no
            # foot sliding); backward plays it in reverse.
            self.t += (-1 if fwd < 0 else 1) * speed * dt / anim_speed
        elif turn:
            self.dancing = False
            self.set_anim("walk")
            self.face += turn * TURN_RPS * dt
            self.t += TURN_STEP_RATE * dt
        elif self.dancing:
            self.set_anim("dance", keep_phase=False)
            self.t += dt
        elif "idle" in self.anims:
            self.set_anim("idle", keep_phase=False)
            self.t += dt
        else:
            changed = False  # no idle animation: hold the last pose

        if changed:
            length = self.anims[self.anim]["length_s"]
            if length > 0:
                self.t %= length
            self.send()
        else:
            self.link.flush()
        self.root.after(TICK_MS, self.tick)

    def hip_motion(self):
        """The clip's hip motion at time t: (forward, left, up) offset in m
        and rotation [w, x, y, z], both in his own frame (x = forward).
        Gazebo ignores the hips' animated position and rotation once the
        plugin positions him, so they're sent along: walking bob and sway,
        crouches, jumps, flips. Up is relative to the spawn height."""
        a = self.anims[self.anim]
        ts = a.get("root_t")
        if not ts:
            return 0.0, 0.0, 0.0, [1.0, 0.0, 0.0, 0.0]
        i = bisect.bisect_right(ts, self.t)
        lo, hi = max(i - 1, 0), min(i, len(ts) - 1)
        f = 0.0 if lo == hi else (self.t - ts[lo]) / ((ts[hi] - ts[lo]) or 1)
        lerp = lambda key, d: (a[key][lo] + f * (a[key][hi] - a[key][lo])) if key in a else d
        q0, q1 = a.get("root_q", [[1, 0, 0, 0]] * len(ts))[lo], a.get("root_q", [[1, 0, 0, 0]] * len(ts))[hi]
        q = [q0[k] + f * (q1[k] - q0[k]) for k in range(4)]  # neighbours share a hemisphere
        n = math.sqrt(sum(v * v for v in q)) or 1
        return lerp("root_x", 0.0), lerp("root_y", 0.0), lerp("root_z", self.root_height) - self.root_height, [v / n for v in q]

    def send(self):
        fwd, left, up, (qw, qx, qy, qz) = self.hip_motion()
        c, s = math.cos(self.face), math.sin(self.face)
        # His facing (a yaw) composed with the clip's hip rotation: q_face * q.
        hw, hz = math.cos(self.face / 2), math.sin(self.face / 2)
        q = (hw * qw - hz * qz, hw * qx - hz * qy, hw * qy + hz * qx, hw * qz + hz * qw)
        self.link.send(self.x + c * fwd - s * left, self.y + s * fwd + c * left, up,
                       self.face, q, self.anim, self.t)
        self.status.config(text=(
            f"x={self.x:7.2f}  y={self.y:7.2f}  facing={math.degrees(self.face) % 360:5.1f}°  "
            f"{'dancing' if self.dancing else self.anim}\nvia {self.link.mode}"))


def main():
    x = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
    y = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    yaw = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    face = yaw + math.pi  # spawn_person's yaw 0 faces -X (the drone); his 0 faces +X
    meta = load_meta()
    ensure_plugin()
    world = find_world()
    link = Link()
    found = link.existing()
    if found:
        x, y, face = found
        print(f"'{ACTOR}' is already in the world (actors can't be removed): "
              f"carrying on from ({x:.2f}, {y:.2f}). Restart the sim for a fresh one.")
    else:
        print(f"Spawning '{ACTOR}' in world '{world}' at ({x}, {y})...")
        spawn(world, meta, x, y)
    print("Drive him from the 'Walk person' window.")
    root = tk.Tk()
    Walker(root, link, meta, x, y, face)
    root.mainloop()


if __name__ == "__main__":
    main()
