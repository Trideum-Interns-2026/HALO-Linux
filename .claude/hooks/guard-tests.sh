#!/usr/bin/env bash
# PreToolUse(Edit|Write|MultiEdit). Reads the tool call as JSON on stdin; exit 2 blocks.
# The rule is AGENTS.md "Tests and CI": never skip, delete or weaken a test.
PY=$(cat <<'PY'
import json
import os
import re
import sys

d = json.load(sys.stdin)
ti = d.get("tool_input") or {}
tool = d.get("tool_name", "")
path = ti.get("file_path", "")


def block(reason):
    print(reason, file=sys.stderr)
    sys.exit(2)


old = ""
if path and os.path.isfile(path):
    with open(path, encoding="utf-8", errors="replace") as fh:
        old = fh.read()


def apply(text, old_s, new_s, replace_all):
    if old_s and old_s in text:
        return text.replace(old_s, new_s) if replace_all else text.replace(old_s, new_s, 1)
    return text


if tool == "Write":
    new = ti.get("content", "")
elif tool == "MultiEdit":
    new = old
    for e in ti.get("edits", []):
        new = apply(new, e.get("old_string", ""), e.get("new_string", ""), e.get("replace_all"))
else:
    new = apply(old, ti.get("old_string", ""), ti.get("new_string", ""), ti.get("replace_all"))


def test_names(text):
    names = set(re.findall(r"^\s*(?:async\s+)?def\s+(test_\w+)", text, re.M))
    names |= {"%s.%s" % m for m in re.findall(r"\bTEST\w*\(\s*(\w+)\s*,\s*(\w+)\s*\)", text)}
    return names


rel = path.replace("\\", "/")

# 1. Removing a test from a test/ directory.
if re.search(r"(?:^|/)tests?/", rel):
    gone = sorted(test_names(old) - test_names(new))
    if gone:
        block("BLOCKED (AGENTS.md Tests and CI): this edit removes test %s; "
              "never delete or weaken a test." % gone[0])

# 2. Adding skip / xfail / disabled markers (docs and .claude files may mention them).
if not rel.endswith(".md") and "/.claude/" not in rel:
    for pat in (r"pytest\.mark\.skip", r"pytest\.mark\.xfail", r"@unittest\.skip",
                r"\bTEST\w*\(\s*\w+\s*,\s*DISABLED_"):
        if len(re.findall(pat, new)) > len(re.findall(pat, old)):
            block("BLOCKED (AGENTS.md Tests and CI): never skip or disable a test (%s)." % pat)

# 3. CI workflows: no removed step or path filter.
if re.search(r"\.github/workflows/[^/]+\.ya?ml$", rel):
    def items(text):
        return {ln.strip() for ln in text.splitlines() if ln.strip().startswith("- ")}
    gone = sorted(items(old) - items(new))
    if gone:
        block("BLOCKED (AGENTS.md Tests and CI): this edit removes a CI step or path filter: %s"
              % gone[0][:60])
PY
)
exec python3 -c "$PY"
