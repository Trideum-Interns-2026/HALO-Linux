# movement

Follows a person: keeps them centred in the camera and at a set distance, and scans for them when
they are lost. It commands the vehicle only over MAVLink (MAVSDK offboard velocity), never through
ROS. This is Lisa's `follow_person` controller ported as a service; the control math is unchanged
(`include/movement/follow_logic.hpp`).

Language: C++ (MAVSDK, control loop), as AGENTS.md recommends for this kind of component.
Maintainer: Lisa.

## Behaviour

Every 100 ms (`kLoopPeriod`) the node, in the original's order:
1. Needs a live vehicle that is armed and in the air. It sends nothing otherwise. With
   `auto_arm_takeoff:=true` it arms and takes off first, as the original did; the default is off,
   so a human arms in QGroundControl (AGENTS.md "Safety and hardware readiness").
2. Starts offboard: a zero velocity setpoint, then offboard mode.
3. **Target seen** (a detection newer than `target_timeout_ms`), with camera intrinsics known:
   - `normalized` = horizontal offset of the person from the image centre, scaled to -1..1
   - yaw rate = `normalized * max_yaw_rate_deg_s`
   - strafe (right) = `0.7 * normalized * max_strafe_m_s`
   - forward = `(distance - safe_distance_m) * 0.35`, capped at `max_forward_m_s`, with the distance
     first clamped to `[min_distance_m, max_distance_m]`; no valid distance means no forward motion
4. **Target lost**: spiral scan. Forward `0.8 * search_forward_m_s`, strafe
   `sin(2 pi * search_orbit_hz * t) * search_strafe_m_s`, yaw `search_yaw_rate_deg_s`, with `t` the
   time since the scan began. It scans until the target returns.
5. Sends the result as a body-frame velocity (altitude is never commanded).

On Ctrl+C or exit it leaves offboard, which returns PX4 to Hold.

### Orbit mode

With `orbit_enabled:=true` (launch: `orbit:=true`) a person in view is circled instead of followed,
and the scan on target loss is unchanged. Off by default.

- **Radius** is `safe_distance_m`, held with the same forward law as follow.
- **Speed** is sideways, `2 pi r / orbit_period_s`, capped by `max_orbit_speed_m_s`.
- **Direction** is counter-clockwise (seen from above) unless `orbit_clockwise`.
- **Yaw is closed-loop on the image only**: proportional plus integral on the pixel offset of the
  person, nothing guessed open-loop. The camera therefore keeps the person centred wherever they
  are, also when they move. Keeping a fixed forward camera on a point while circling it means the
  heading turns once per lap; that is geometry, and the controller finds the rate itself. The
  integral is cleared whenever the target is lost.
- Without a valid range it only keeps the person centred (no orbit motion).

The default lap is 30 s: at 6 m that is 1.26 m/s sideways, above the follow strafe limit
(0.45 m/s), so orbit has its own cap (`max_orbit_speed_m_s`, default 1.5, hard cap 2.0). Follow and
the scan keep their old limits. A tighter radius or a longer period lowers the speed; a lap that
would need more than the cap simply takes longer.

### Search around the last known direction

With `last_known_search_s > 0` (default 0 = off, the original behaviour), losing the target no
longer starts the spiral scan at once. The vehicle holds its position and sweeps its nose by
`last_known_sweep_deg` (60) either side of where the person was last seen, at
`last_known_rate_deg_s` (20), first toward the side they were drifting to (the pixel offset when it
was off-centre, otherwise the last turn direction). Only yaw is commanded. If that finds nothing
within `last_known_search_s`, the original spiral scan takes over, so it still never gives up
(unless `search_timeout_s` says so). Seeing the person again ends it and follow or orbit resumes.
The spiral drives away from the last known position and, at 25 deg/s, takes about 14 s per turn;
this keeps the camera on the place the person vanished.

### Boxes cut off by the frame edge

A person whose box touches the top or bottom of the image (feet below the frame when the drone is
high and close) has a short box, so the pinhole range comes out far too long and the drone would
drive forward and pitch the target out of view. Such a box counts as "no valid range": no forward
motion, only centring. Needs the image height from `camera_info`.

### Input

The original read a pixel position and a distance from the old detector's topics, which no longer
exist. Movement now reads Perception's `perception_msgs/Detection` and derives the same two
numbers: position = box centre x, distance = `person_height_m * fy / box_height_px` from
`camera_info` (what the old detector computed).

### Deliberate differences from the original

Each is a bug or a repo-rule fix, and each has a test.
- Offboard is restarted after a landing and new take-off (the original never cleared its flag).
- A person in view but no `camera_info` yet holds still instead of running the scan.
- Search speeds are clamped to the follow limits, and every limit has a hard cap in code
  (`follow_logic.hpp`), whatever the parameters say.
- The MAVSDK URL is a parameter (default is the SITL value); camera topic and QoS follow the
  Perception conventions; timing uses ROS time.
- Arming and take-off are off by default (see above).
- A lost vehicle link sends nothing and re-enters offboard when it returns.
- `search_timeout_s` (default `0`, scan forever like the original) can hand back to Hold after that
  long, until the target is seen again.

## Interface

| Direction | What | Notes |
| --- | --- | --- |
| in | `/perception/detection` (`perception_msgs/Detection`) | pixel box of the person |
| in | `/camera/front/camera_info` (`sensor_msgs/CameraInfo`) | image centre, width, focal lengths |
| in | vehicle telemetry | MAVLink: armed, in air |
| out | velocity setpoints, offboard start/stop | MAVLink only |

No topics are published.

## Parameters

| Name | Default | Meaning |
| --- | --- | --- |
| `mavsdk_url` | `udpin://0.0.0.0:14540` | MAVSDK connection (SITL); change for HITL / hardware |
| `detection_topic` | `/perception/detection` | person box input |
| `camera_info_topic` | `/camera/front/camera_info` | intrinsics input |
| `safe_distance_m` | `6.0` | distance to hold from the person |
| `min_distance_m` / `max_distance_m` | `4.0` / `9.0` | clamp applied to the measured distance (hard caps 2 / 30) |
| `max_forward_m_s` | `0.7` | forward speed limit (hard cap 2.0) |
| `max_strafe_m_s` | `0.45` | sideways speed limit (hard cap 1.5) |
| `max_yaw_rate_deg_s` | `45.0` | yaw rate limit (hard cap 90) |
| `target_timeout_ms` | `800` | a detection older than this counts as lost |
| `person_height_m` | `1.9` | height used to turn the box into a distance (the test person) |
| `search_yaw_rate_deg_s` | `25.0` | yaw rate while scanning |
| `search_forward_m_s` | `0.35` | scan forward speed (times 0.8) |
| `search_strafe_m_s` | `0.45` | scan sideways amplitude |
| `search_orbit_hz` | `0.25` | scan sideways frequency (max 1) |
| `last_known_search_s` | `0.0` | time to sweep around the last known direction before the spiral (0 = off, max 60) |
| `last_known_sweep_deg` | `60.0` | sweep half-angle (max 180) |
| `last_known_rate_deg_s` | `20.0` | sweep yaw rate (capped at `max_yaw_rate_deg_s`) |
| `orbit_enabled` | `false` | circle the person instead of following |
| `orbit_period_s` | `30.0` | time for one lap (5-600 s) |
| `orbit_clockwise` | `false` | direction, seen from above |
| `max_orbit_speed_m_s` | `1.5` | sideways speed cap while orbiting (hard cap 2.0) |
| `search_timeout_s` | `0.0` | `0` = scan until found; otherwise hand back to Hold after this long |
| `auto_arm_takeoff` | `false` | arm and take off by itself, as the original did |

## Run

```
fly                                  # sim; arm and take off in QGroundControl
perceive                             # detections
movement                             # ros2 launch movement movement.launch.py
ros2 run movement movement_node --ros-args -p safe_distance_m:=8.0
```

## Test

```
cd /opt/ros2_ws && colcon test --packages-select movement && colcon test-result --verbose
```
CI: `.github/workflows/movement-tests.yml`.

## Tuning

The gains are parameters (defaults are Lisa's originals): `forward_gain`, `lateral_gain`,
`yaw_gain`, `orbit_yaw_ki`, plus optional acceleration limits `max_forward_accel_m_s2`,
`max_strafe_accel_m_s2`, `max_yaw_accel_deg_s2` (0 = off) that smooth the commands. Hard ceilings
in `follow_logic.hpp` still apply.

`config/sim_tuned.yaml` holds values tuned **for the simulator**, for smooth motion. Load it with
`ros2 launch movement movement.launch.py params_file:=<path to config/sim_tuned.yaml>`
(`sim_check --tuned` does). They may need retuning for hardware, a different camera rate, or other
PX4 gains.

`movement_tune` (built with the package, no ROS or sim needed) runs the real controller against a
model of the drone and camera and prints the smoothest settings that still keep the person within
12 degrees of the image centre, hold the range within 1 m, settle a step in 12 s and keep orbit laps
on time, on slow, nominal and fast vehicle models:

```bash
ros2 run movement movement_tune              # search
ros2 run movement movement_tune 1.0 0.35 0.5 20 0.3 0.3 30   # score one set: yaw lat fwd ki a_fwd a_str a_yaw
```

The plant numbers at the top of `tools/tune_movement.cpp` (velocity and yaw lag, detection rate
and latency) are assumptions: fit them to a flight trace (`sitl_vehicle trace`) and rerun. Finding:
the original forward gain 0.35 cannot hold range on a person walking at 0.5 m/s (P-only, steady lag
about 1.4 m); 0.5 can.
