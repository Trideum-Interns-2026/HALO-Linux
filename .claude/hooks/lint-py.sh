#!/usr/bin/env bash
# PostToolUse(Edit|Write|MultiEdit). Lints an edited .py file (AGENTS.md "Microservice rules":
# lines <= 99 chars). Reports to the model, never blocks.
f=$(python3 -c 'import json,sys; print((json.load(sys.stdin).get("tool_input") or {}).get("file_path",""))')
[[ "$f" == *.py && -f "$f" ]] || exit 0

command -v ruff >/dev/null 2>&1 || pip install --quiet --break-system-packages ruff >/dev/null 2>&1
command -v ruff >/dev/null 2>&1 || exit 0

out=$(ruff check --line-length 99 --select E501,F "$f" 2>&1) && exit 0   # clean: stay silent
[[ -n "$out" ]] || exit 0
OUT="$out" python3 -c '
import json, os
print(json.dumps({"hookSpecificOutput": {"hookEventName": "PostToolUse",
                                         "additionalContext": "ruff:\n" + os.environ["OUT"]}}))'
exit 0
