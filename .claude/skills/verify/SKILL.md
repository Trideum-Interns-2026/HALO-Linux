---
name: verify
description: Verify the working tree, or a git ref range, against AGENTS.md
argument-hint: "[git ref range] [--opus]"
user-invocable: true
---
Arguments: $ARGUMENTS. `--opus` is a flag, not a ref; the rest (if any) is the ref range.

1. Spawn **verifier** (a subagent, so its output stays out of this context) on the working tree, or on
   the ref range if given.
2. Run `git diff <range>` (working tree if empty). Run **safety-reviewer** with the full diff
   (it has no shell) ONLY when the diff touches `Movement/`, `Src/`, `Include/`, a `.msg` file, or the
   diff text matches `mavsdk|mavlink|offboard|arm|takeoff|goto`. Otherwise print one line saying why it
   was skipped. If `--opus` was given, run that review with the `opus` model regardless of profile.
3. Print the verifier table first, then the safety findings and verdict (or the skip line).

Edit nothing.
