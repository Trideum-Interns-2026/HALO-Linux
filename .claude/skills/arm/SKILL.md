---
name: arm
description: Arm / take off / land / disarm the SITL vehicle (human-triggered, simulator only)
argument-hint: arm | takeoff [alt_m] | land | disarm | status
disable-model-invocation: true
user-invocable: true
---
Run `/workspace/scripts/sitl_vehicle $ARGUMENTS` and report its output. SITL only: the script
refuses non-local endpoints, and takeoff is capped at 10 m. Never run it against a real vehicle.

`fly` must already be running. The helper uses PX4's API port 14580, so it can run while
`movement` holds 14540. Run `sitl_vehicle link &` for the whole flight so PX4 keeps seeing a ground
station (otherwise it fails over to Return). Extra actions: `position` and `status` (shows the mode).

The repo guard hook (`.claude/hooks/guard-bash.sh`) blocks the words arm/takeoff in agent Bash
calls on purpose. If it blocks this command, do not edit or work around the hook: tell the user to
run `! sitl_vehicle <action>` themselves.
