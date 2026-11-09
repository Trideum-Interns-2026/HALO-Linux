---
name: feature
description: Plan, build, test, verify and review a change end to end
argument-hint: <what to build or fix>
user-invocable: true
---
Task: $ARGUMENTS

Read `.claude/profile` (first line, `max` or `pro`) and apply:
- max: run `plan-critic` always; team cap 5; team threshold 2 independent items.
- pro: run `plan-critic` only if any item is `[careful]` or there are 3+ items; team cap 3; team
  threshold 3 independent items.

Tell every agent to read AGENTS.md first.

1. **Plan.** Run the `plan` skill on the task. If the output starts with `STOP`, print it and end.
   Run **plan-critic** per the profile rule; send any gaps back to the planner once. Show the plan and
   wait for a yes. If the plan touches `Movement/`, suggest plan mode first.
   1b. If the plan has fewer independent items than the profile's team threshold: spawn the writers as
   unnamed subagents (**implementer-fast** for `[fast]`, **implementer** for `[careful]`,
   **test-writer** for tests), run **verifier** as a fork, and skip to step 6.
2. **Tasks.** Create tasks `impl: <files>`, `test: <files>`, `verify: all`, `safety: all`,
   `docs: all`, with dependencies in that order.
3. **Team.** Spawn named teammates (`fast-1`, `careful-1`, `tests-1`, ...) from `implementer-fast`,
   `implementer` and `test-writer`, up to the profile's cap. The rest self-claim.
4. **Wait.** Do not implement yourself. Nudge a silent teammate once.
5. **Verify.** **verifier** as a teammate on `verify: all`. On BLOCKED rows, ask the user once whether to
   run **sim-operator** on them (it costs a 2 to 4 minute sim run; in full SITL it may arm and fly
   on its own).
6. **Safety.** **safety-reviewer**, forked, with the same conditional as `/verify` (only when the diff
   touches `Movement/`, `Src/`, `Include/`, a `.msg` file, or matches
   `mavsdk|mavlink|offboard|arm|takeoff|goto`; otherwise print why it was skipped).
7. **Docs.** **docs-sync** as a teammate with the list of changed files. Shut the teammates down.

Finish by printing, not committing:
- a commit message: imperative one-line subject, body naming the AGENTS.md sections touched;
- a PR description with the "Before finishing any change" checklist filled from the verifier table.

Do not run `git commit` or `git push`.
