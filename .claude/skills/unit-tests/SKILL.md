---
name: unit-tests
description: Default way to prove any change in this repo; load before writing or running tests.
---
Rules live in AGENTS.md "Tests and CI". This is the procedure.

## Layer 1: logic tests (no ROS)
Test the pure functions, parametrized over input shapes, with edge cases: exact thresholds, limits and
clipping, several candidates, nothing found, round trips. Pattern: `Perception/test/test_detector.py`
(a `FRAME_SIZES` list fed to `@pytest.mark.parametrize`, helper `make_raw(...)`, call static methods
directly).

## Layer 2: node tests (real node, fake outside world)
Pattern: `Perception/test/test_node.py`.
1. A fixture that calls `rclpy.init()`, builds the real node, sets `node.pub = MagicMock()`, yields it,
   then `destroy_node()` and `rclpy.shutdown()`.
2. Replace the external call: `node._infer = lambda blob: fake_output(0.9)`.
3. Assert, in this order: the header is copied from the input; the values are right; nothing is
   published when nothing is found (`node.pub.publish.assert_not_called()`); a failing external call
   does not raise and publishes nothing; every parameter has its documented default
   (`node.get_parameter(...)`).

## C++
`ament_cmake_gtest`; put the pure functions in a header so `TEST(Suite, Name)` can call them without
ROS; node tests construct the node with a fake behind its interface (see `fake-externals`).

## Motion code (Movement)
One test per limit, showing it is enforced. Loss of target and loss of connection each end in Hold, and
so does exit.

## Run
```bash
cd /opt/ros2_ws && colcon build --packages-select <pkg>_msgs <pkg> --symlink-install && source install/setup.bash
cd /opt/ros2_ws && colcon test --packages-select <pkg>_msgs <pkg> && colcon test-result --verbose
cd /workspace/<Pkg> && python3 -m pytest test/ -v        # Python packages, faster
```

## Never
Skip, delete, deselect or weaken a test to get green.

If the claim cannot be unit-tested, say so explicitly and hand it to the verifier as BLOCKED. Do not
start the sim from here; only the sim-operator does that.
