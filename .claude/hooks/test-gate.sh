#!/usr/bin/env bash
# TaskCompleted / TeammateIdle / SubagentStop gate. Exit 2 + pytest summary on stderr if a touched
# package with a test/ dir is red. Reads the hook JSON on stdin.
input=$(cat)
read -r event subject < <(printf '%s' "$input" | python3 -c '
import json, sys
try:
    d = json.load(sys.stdin)
except Exception:
    d = {}
print(d.get("hook_event_name", ""), (d.get("task_subject") or "").replace("\n", " "))')
if [[ "$event" == "TaskCompleted" && "$subject" =~ ^(plan|verify|safety|docs): ]]; then
    exit 0
fi
root=$(git rev-parse --show-toplevel 2>/dev/null) || exit 0
cd "$root" || exit 0

pkgs=$( { git diff --name-only; git diff --name-only --cached; git ls-files --others --exclude-standard; } \
        | sed 's#/.*##' | sort -u | while read -r top; do
            [[ -n "$top" && -d "$top" && -f "$top/package.xml" ]] && echo "$top"
        done)
fail=""
for pkg in $pkgs; do
    [[ -d "$pkg/test" ]] || continue
    lower=$(echo "$pkg" | tr '[:upper:]' '[:lower:]')
    if [[ -f "$pkg/setup.py" ]]; then
        out=$(bash -c "source /opt/ros/jazzy/setup.bash; \
                       source /opt/ros2_ws/install/setup.bash 2>/dev/null; \
                       cd '$root/$pkg' && timeout 600 python3 -m pytest test/ -q" 2>&1); rc=$?
    elif [[ -d "/opt/ros2_ws/src/$pkg" ]]; then
        out=$(bash -c "source /opt/ros/jazzy/setup.bash; cd /opt/ros2_ws && \
                       timeout 900 colcon test --packages-select $lower >/dev/null 2>&1; \
                       colcon test-result" 2>&1); rc=$?
        [[ "$out" == *"0 failures"* || "$out" == *"Summary: "*" 0 errors, 0 failures"* ]] && rc=0
        [[ "$out" =~ ([1-9][0-9]*)\ (errors|failures) ]] && rc=1
    else
        continue
    fi
    if [[ $rc -ne 0 ]]; then
        fail+="[$pkg] FAIL"$'\n'"$(printf '%s' "$out" | tail -n 5)"$'\n'
    fi
done
if [[ -n "$fail" ]]; then
    printf '%s' "$fail" >&2
    exit 2
fi
exit 0
