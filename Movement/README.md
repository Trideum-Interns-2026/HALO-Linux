# movement

Follows a tracked person: keeps them centred in the camera and at a set distance, and scans for them
when they are lost. It commands the vehicle only over MAVLink (MAVSDK offboard velocity), never through
ROS. The follow and orbit control math is in `include/movement/follow_logic.hpp`.

Language: C++ (MAVSDK, control loop).
Maintainer: Lisa.

## Behaviour

Every 100 ms (`kLoopPeriod`) the node, in this order:
1. Needs a live vehicle that is armed and in the air. It sends nothing otherwise. With
   `auto_arm_takeoff:=true` it arms and takes off first; the default is off, so a human arms in
   QGroundControl.
2. Starts offboard: a zero velocity setpoint, then offboard mode.
3. **Target seen** (tracking and drone position both newer than `target_timeout_ms`, drone heading
   known, camera intrinsics known; see "Input: tracked position"):
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
the scan keep the follow limits. A tighter radius or a longer period lowers the speed; a lap that
would need more than the cap simply takes longer.

### Search around the last known direction

With `last_known_search_s > 0` (default 0 = off), losing the target does not start the spiral scan
at once. The vehicle holds its position and sweeps its nose by `last_known_sweep_deg` (60) either
side of where the person was last seen, at `last_known_rate_deg_s` (20), first toward the side they
were drifting to (the pixel offset when it was off-centre, otherwise the last turn direction). Only
yaw is commanded. If that finds nothing within `last_known_search_s`, the spiral scan takes over, so
it still never gives up (unless `search_timeout_s` says so). Seeing the person again ends it and
follow or orbit resumes. The spiral drives away from the last known position and, at 25 deg/s, takes
about 14 s per turn; this keeps the camera on the place the person vanished.

### Input: tracked position

Movement follows Tracking's person position, not the camera image. Each control step it turns three
inputs into the pixel column and range the controller uses, then runs the follow or orbit controller:

- person: `/tracking/position` (`tracking_msgs/TrackingState`, LLA)
- drone: `/geolocation/position` (`geolocation_msgs/GeolocationState`, LLA)
- drone heading: MAVSDK telemetry `attitude_euler().yaw_deg` (degrees from true north, clockwise)

Synthesis (`include/movement/target_geometry.hpp`):
1. North and east offset of the person from the drone (WGS84 radii at the mean latitude).
2. World bearing `atan2(east, north)`, minus the heading, wrapped to (-180, 180].
3. Pixel column `cx + fx * tan(relative bearing)` from `camera_info` (bearing clamped to +-89 deg).
4. Range = horizontal ground distance (altitude is ignored, not the slant range).

Rules:
- **Out of view**: the range is given only while the column is inside the image. A target behind or
  beside the drone gets no range, so in follow mode it only yaws (forward and strafe are not
  commanded: `min_distance_m` cannot be enforced without a range). With the acceleration limits and
  smoothing of `config/sim_tuned.yaml` the forward speed left over from before decays to zero over
  about 2.5 s after a target swings behind; it never increases. Orbit mode is unchanged.
- **Heading freshness**: the MAVSDK attitude timestamp must keep advancing. If it stops for longer
  than `heading_timeout_s` (the stream died while the link stayed up) the heading counts as
  unknown and the lost-target search runs; a new link also starts without a heading.
- **Too close**: nearer than 0.5 m the bearing is meaningless, so the target is taken as centred with
  no range (hover).
- **Error gate**: `max_target_error_m` (0 = off) drops a tracking message whose 3-sigma ECEF error
  norm is above the limit; non-finite or negative errors are always dropped. A dropped message does
  not refresh the target, so it goes stale and the search starts.
- **Fresh** means tracking and geolocation were both received (node time, never the header stamp)
  within `target_timeout_ms`, the heading is known and `camera_info` is usable. Anything else runs
  the lost-target search above.
- The camera is assumed to face along the body axis (no mount-yaw parameter), so the heading is the
  camera heading. `camera_info` supplies only intrinsics (centre x, focal x, width).
- The follow distance is the horizontal ground range to the tracked person.

Other behaviour:
- Offboard is restarted after a landing and new take-off.
- Search speeds are clamped to the follow limits, and every limit has a hard cap in code
  (`follow_logic.hpp`), whatever the parameters say.
- A lost vehicle link sends nothing and re-enters offboard when it returns.
- `search_timeout_s` (default `0`, scan forever) can hand back to Hold after that long, until the
  target is seen again.
- Timing uses ROS time; arming and take-off are off by default.

## Interface

| Direction | What | Notes |
| --- | --- | --- |
| in | `/tracking/position` (`tracking_msgs/TrackingState`) | person LLA + ECEF 3-sigma error |
| in | `/geolocation/position` (`geolocation_msgs/GeolocationState`) | drone LLA + ECEF 3-sigma error |
| in | `/camera/front/camera_info` (`sensor_msgs/CameraInfo`) | image centre, width, focal length x |
| in | vehicle telemetry | MAVLink: armed, in air, heading |
| out | velocity setpoints, offboard start/stop | MAVLink only |

No topics are published.

## Parameters

| Name | Default | Meaning |
| --- | --- | --- |
| `mavsdk_url` | `udpin://0.0.0.0:14540` | MAVSDK connection (SITL); change for HITL / hardware |
| `heading_timeout_s` | `1.0` | attitude stream older than this (timestamp not advancing) = heading unknown; clamped to 0.1-10 s; read by `main.cpp`, not the node |
| `tracking_topic` | `/tracking/position` | person position input |
| `geolocation_topic` | `/geolocation/position` | drone position input |
| `max_target_error_m` | `0.0` | drop a tracking message whose 3-sigma error norm exceeds this (0 = off, max 1000) |
| `camera_info_topic` | `/camera/front/camera_info` | intrinsics input |
| `safe_distance_m` | `6.0` | distance to hold from the person |
| `min_distance_m` / `max_distance_m` | `4.0` / `9.0` | clamp applied to the measured distance (hard caps 2 / 30) |
| `max_forward_m_s` | `0.7` | forward speed limit (hard cap 2.0) |
| `max_strafe_m_s` | `0.45` | sideways speed limit (hard cap 1.5) |
| `max_yaw_rate_deg_s` | `45.0` | yaw rate limit (hard cap 90) |
| `target_timeout_ms` | `800` | a tracking or geolocation message older than this counts as lost |
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
| `auto_arm_takeoff` | `false` | arm and take off by itself; leave off, a human arms the vehicle |

## Run

```
simstack up --gui                    # sim stack: fly, simlinks, perception, geolocation, tracking, ui
                                     # (or start those by hand, see the root README)
# arm and take off in QGroundControl, or: sitl_vehicle arm / sitl_vehicle takeoff 5
movement                             # ros2 launch movement movement.launch.py
ros2 run movement movement_node --ros-args -p safe_distance_m:=8.0
```

It needs the whole stack: simlinks, geolocation, tracking (which needs perception) and the vehicle
airborne. Without fresh tracking and geolocation it runs the target-lost search.

## Test

```
cd /opt/ros2_ws && colcon test --packages-select movement && colcon test-result --verbose
```
CI: `.github/workflows/movement-tests.yml`.

## Tuning

The gains are parameters: `forward_gain`, `lateral_gain`, `yaw_gain`, `orbit_yaw_ki`, plus optional
acceleration limits `max_forward_accel_m_s2`, `max_strafe_accel_m_s2`, `max_yaw_accel_deg_s2` (0 = off)
that smooth the commands, and `command_smoothing_s` (low-pass time constant after those limits,
0 = off, at most 2 s) that rounds off the acceleration steps so jerk stays low. The low-pass only lags
the command and never exceeds a speed or acceleration limit, but it adds delay to the control loop:
too large a value makes the follower overshoot or lose the person, so change it only together with
the gains (`movement_tune` scores that). Hard ceilings in `follow_logic.hpp` still apply.

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
ros2 run movement movement_tune 1.0 0.35 0.5 20 0.3 0.3 30 0.2   # score one set: yaw lat fwd ki a_fwd a_str a_yaw [smoothing_s]
```

The plant numbers at the top of `tools/tune_movement.cpp` (velocity and yaw lag, detection rate
and latency) are assumptions: fit them to a flight trace and rerun. Finding: the P-only forward gain
0.35 cannot hold range on a person walking at 0.5 m/s (steady lag about 1.4 m); 0.5 can.
