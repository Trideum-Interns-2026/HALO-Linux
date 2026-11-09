---
name: sim
description: Run one headless sim check through the sim-operator and report
argument-hint: [check to perform]
user-invocable: true
context: fork
agent: sim-operator
---
Run this check: $ARGUMENTS

If empty, the check is: detections arrive on `/perception/detection` with a spawned person.

Report your output only. Do not edit anything.
