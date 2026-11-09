#!/usr/bin/env bash
# Stop hook. Runs the tests of every package with changed files so a "tests pass" claim is
# backed by output (AGENTS.md "Tests and CI"). Never blocks.
root=$(git rev-parse --show-toplevel 2>/dev/null) || exit 0
cd "$root" || exit 0

pkgs=$( { git diff --name-only; git diff --name-only --cached; git ls-files --others --exclude-standard; } \
        | sed 's#/.*##' | sort -u | while read -r top; do
            [[ -n "$top" && -d "$top" && -f "$top/package.xml" ]] && echo "$top"
        done)
[[ -n "$pkgs" ]] || exit 0

report=""
for pkg in $pkgs; do
    [[ -d "$pkg/test" ]] || continue
    lower=$(echo "$pkg" | tr '[:upper:]' '[:lower:]')
    if [[ -f "$pkg/setup.py" ]]; then
        out=$(bash -c "source /opt/ros/jazzy/setup.bash; \
                       source /opt/ros2_ws/install/setup.bash 2>/dev/null; \
                       cd '$root/$pkg' && timeout 600 python3 -m pytest test/ -q" 2>&1 | tail -n 4)
    elif [[ -d "/opt/ros2_ws/src/$pkg" ]]; then
        out=$(bash -c "source /opt/ros/jazzy/setup.bash; cd /opt/ros2_ws && \
                       timeout 900 colcon test --packages-select $lower >/dev/null 2>&1; \
                       colcon test-result" 2>&1 | tail -n 3)
    else
        out="C++ package not linked into /opt/ros2_ws/src; tests not run."
    fi
    report+="[$pkg] ${out}"$'\n'
done
[[ -n "$report" ]] || exit 0

printf '%s' "$report"
REPORT="$report" python3 -c '
import json, os
print(json.dumps({"systemMessage": "test-touched:\n" + os.environ["REPORT"]}))'
exit 0
