---
name: new-service
description: Scaffold a new microservice package
argument-hint: <Name> <owner>
user-invocable: true
---
Arguments: $ARGUMENTS (first word = service name, second = owner).

Read `.claude/profile` (first line, `max` or `pro`) and apply:
- max: run `plan-critic` always; team cap 5; team threshold 2 independent items.
- pro: run `plan-critic` only if any item is `[careful]` or there are 3+ items; team cap 3; team
  threshold 3 independent items.

Tell every agent to read AGENTS.md first.

1. **Plan.** Confirm the folder and package names with the user first (AGENTS.md "Package names") and
   wait. Then run the `plan` skill, telling the planner to load
   `.claude/skills/new-service-scaffold/SKILL.md`. If the output starts with `STOP`, print it and end.
   Run **plan-critic** per the profile rule. Show the plan and wait for a yes. Suggest plan mode if it
   touches Movement.
   1b. If the plan has fewer independent items than the team threshold: spawn the writers as unnamed
   subagents, run **verifier** as a fork, and skip to step 6.
2. **Tasks.** Create `impl: <files>`, `test: <files>`, `verify: all`, `safety: all`, `docs: all`, with
   dependencies in that order.
3. **Team.** Spawn named teammates (`fast-1`, `careful-1`, `tests-1`, ...) from `implementer-fast`,
   `implementer`, `test-writer`, up to the profile's cap. The rest self-claim.
4. **Wait.** Do not implement yourself. Nudge a silent teammate once.
5. **Verify.** **verifier** as a teammate on `verify: all`. On BLOCKED rows, ask the user once whether
   to run **sim-operator**.
6. **Safety.** **safety-reviewer**, forked, with the same conditional as `/verify`.
7. **Docs.** **docs-sync** as a teammate for README, `post-create.sh` and AGENTS.md tables. Shut the
   teammates down.

Print the verifier table, a commit message and a PR description. Do not commit or push.
