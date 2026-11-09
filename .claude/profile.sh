#!/usr/bin/env bash
# Usage: profile.sh max|pro. Rewrites model: in planner, verifier, safety-reviewer and .claude/profile.
set -e
d=$(cd "$(dirname "$0")" && pwd)
case "${1:-}" in
    max) model=opus ;;
    pro) model=sonnet ;;
    *) echo "usage: profile.sh max|pro" >&2; exit 2 ;;
esac
for a in planner verifier safety-reviewer; do
    sed -i "0,/^model:.*/s//model: $model/" "$d/agents/$a.md"
done
echo "$1" > "$d/profile"
