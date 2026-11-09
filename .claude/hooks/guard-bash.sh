#!/usr/bin/env bash
# PreToolUse(Bash). Reads the tool call as JSON on stdin; exit 2 + one line on stderr blocks.
# Rules live in AGENTS.md ("Commands", "Safety and hardware readiness"); this only enforces them.
PY=$(cat <<'PY'
import json
import os
import re
import sys

cmd = (json.load(sys.stdin).get("tool_input") or {}).get("command", "")


def block(reason):
    print(reason, file=sys.stderr)
    sys.exit(2)


# 1. colcon only from the ROS workspace (AGENTS.md, Commands).
for line in cmd.splitlines():
    m = re.search(r"\bcolcon\b", line)
    if m and "--base-paths" not in line and "cd /opt/ros2_ws" not in line[: m.start()]:
        block("BLOCKED (AGENTS.md Commands): never run colcon in /workspace; "
              "use `cd /opt/ros2_ws && colcon ...` or --base-paths.")

# 2. Motion and hardware are never allowed; sim bring-up only for the sim-operator.
CMD_POS = r"(?:^|[;&|(]\s*)(?:[A-Za-z_]+=\S*\s+)*"
NEVER = [
    r"\barm\b", r"\btakeoff\b", r"\boffboard\b",
    r"scripts/move", r"scripts/follow_person", r"scripts/track",
    CMD_POS + r"(?:move|track|follow_person)(?=\s|$|;|&)",
    r"/dev/tty", r"/dev/serial",
]
SIM = [r"scripts/fly", CMD_POS + r"fly(?=\s|$|;|&)", r"ros2 launch", r"gz sim"]
operator = os.environ.get("HALO_SIM_OPERATOR") == "1" or re.search(
    r"(?:^|[;&|(\s])HALO_SIM_OPERATOR=1(?=\s|;|&|$)", cmd)

# 2a. Belt and braces to the settings.json deny list: no push, docker or recursive force delete.
for pat, what in ((CMD_POS + r"git\s+push\b", "git push"), (CMD_POS + r"docker\b", "docker"),
                  (CMD_POS + r"rm\s+(?:-\S*[rR]\S*f|-\S*f\S*[rR]|-[rR]\s+-f|-f\s+-[rR])", "rm -rf")):
    if re.search(pat, cmd, re.M):
        block("BLOCKED: %s is not allowed for agents; ask the user to run it." % what)

# 2b. SITL only, enforced mechanically: the flight helpers get no endpoint override, no
# non-loopback address, no serial device, and only run when a sim model is set and no process
# holds a serial port.
if re.search(r"\bsitl_vehicle\b|\bsim_check\b", cmd):
    if re.search(r"--endpoint\b|--url\b|serial://|/dev/tty|/dev/serial", cmd):
        block("BLOCKED (SITL only): sitl_vehicle and sim_check take no --endpoint/--url "
              "and no serial device.")
    for ip in re.findall(r"\b(\d{1,3}(?:\.\d{1,3}){3})\b", cmd):
        if not ip.startswith("127.") and ip != "0.0.0.0":
            block("BLOCKED (SITL only): non-loopback address %s." % ip)
    if not (os.environ.get("PX4_SIM_MODEL") or re.search(r"\bPX4_SIM_MODEL=\S+", cmd)):
        block("BLOCKED (SITL only): PX4_SIM_MODEL is unset, so there is no sim to talk to.")
    import subprocess
    try:
        held = subprocess.run(["pgrep", "-f", "/dev/tty"], capture_output=True, text=True,
                              timeout=5).stdout.split()
    except Exception:
        held = []
    held = [p for p in held if p != str(os.getpid())]
    if held:
        block("BLOCKED (SITL only): a process holds a serial port (pid %s); "
              "a real vehicle may be attached." % held[0])

# The sim-operator may call the SITL helper (it refuses non-local endpoints, caps takeoff at 10 m).
# Only that exact call is exempt; --endpoint or any other arm/takeoff text still blocks.
checked = cmd
if operator:
    checked = re.sub(r"(?:/workspace/)?scripts/sitl_vehicle\s+(?:arm|takeoff|land|disarm|status)"
                     r"(?:\s+\d+(?:\.\d+)?)?(?=\s*(?:$|[;&|]))", "", cmd, flags=re.M)

for pat in NEVER:
    if re.search(pat, checked, re.M):
        block("BLOCKED (AGENTS.md Safety and hardware readiness): no arm, takeoff, "
              "offboard, motion programs or serial devices (matched %r)." % pat)
if not operator:
    for pat in SIM:
        if re.search(pat, cmd, re.M):
            block("BLOCKED: sim bring-up is for the sim-operator agent only "
                  "(HALO_SIM_OPERATOR=1); prove it with a unit test (matched %r)." % pat)
else:
    if re.search(r"ros2 launch\b(?!\s+(?:perception|geolocation|tracking|movement)\b)", cmd):
        block("BLOCKED: the sim-operator may only launch the HALO service packages "
              "(perception, geolocation, tracking, movement).")
    if re.search(r"\bgz sim\b", cmd):
        block("BLOCKED: the sim-operator starts the sim through scripts/fly, not gz sim.")

# 3. Tests are never deselected (AGENTS.md, Tests and CI).
if re.search(r"\bpytest\b", cmd):
    if re.search(r"--deselect\b", cmd):
        block("BLOCKED (AGENTS.md Tests and CI): never deselect tests.")
    if re.search(r"--ignore(?:-glob)?(?:=|\s+)\S*tests?/", cmd):
        block("BLOCKED (AGENTS.md Tests and CI): never ignore a test/ directory.")
    for m in re.finditer(r"(?:^|\s)-k\s*(?:\"([^\"]*)\"|'([^']*)'|(\S+))", cmd):
        expr = next(g for g in m.groups() if g is not None)
        if re.search(r"\bnot\b", expr):
            block("BLOCKED (AGENTS.md Tests and CI): -k must select tests, not deselect them.")
PY
)
exec python3 -c "$PY"
