#!/usr/bin/env bash
# SessionStart. Prints short project context as additionalContext. Never blocks.
root=$(git rev-parse --show-toplevel 2>/dev/null || echo /workspace)
cd "$root" 2>/dev/null || exit 0
source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
if [[ ! -f /opt/ros2_ws/install/setup.bash ]]; then
    (cd /opt/ros2_ws && colcon build \
        --packages-select perception_msgs perception geolocation_msgs geolocation \
            tracking_msgs tracking movement \
        --symlink-install) >/dev/null 2>&1
fi
profile=$(head -n1 .claude/profile 2>/dev/null || echo unknown)
branch=$(git branch --show-current 2>/dev/null)
status=$(git status --short 2>/dev/null | head -n 8)
tested=$(for d in */test; do [[ -d "$d" ]] && echo "${d%/test}"; done | paste -sd' ')
perc=$( (source /opt/ros2_ws/install/setup.bash 2>/dev/null; cd Perception 2>/dev/null && \
         timeout 120 python3 -m pytest test/ -q 2>&1 | tail -n 1) )
CTX="profile: $profile
branch: $branch
git status --short (first 8):
$status
packages with test/: $tested
Perception pytest: $perc" python3 -c '
import json, os
print(json.dumps({"hookSpecificOutput": {"hookEventName": "SessionStart",
                                         "additionalContext": os.environ["CTX"]}}))'
exit 0
