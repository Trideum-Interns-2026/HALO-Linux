---
name: fake-externals
description: Load when code calls Triton, MAVSDK or publishes ROS messages, to make it testable without them.
---
AGENTS.md "Microservice rules" requires every external call behind one method or interface so a test
can replace it. This is how to fake each kind.

## Triton (Python)
- Put the call in one method (`_infer(blob)`); the test assigns `node._infer = lambda blob: raw`.
- Fake output has the model's shape: `np.zeros((1, 84, 8400), dtype=np.float32)`, box in rows 0-3 of a
  column, person score in **row 4** of that column (`raw[0, 4, 123] = 0.9`).
- Failure path: `raise InferenceServerException("Deadline Exceeded")` from the fake. The node must log
  and skip, never raise, and publish nothing.

## Publishers
`node.pub = MagicMock()`; assert on `node.pub.publish.call_args.args[0]`.

## MAVSDK / vehicle (C++)
- One small interface the node owns (for example `VehicleInterface`: state, goto or velocity command,
  hold). The real class wraps MAVSDK; nothing else includes MAVSDK headers.
- The test's fake records every command in a struct the test can read and returns canned state. Pattern:
  `FakeVehicle` in `Movement/test/test_node.cpp`.
- Always assert Hold on exit, on lost target and on lost connection: count the fake's hold calls and
  check the node stops commanding afterwards.
- Drive time from the test (pass the clock into the control step) so no test sleeps.
