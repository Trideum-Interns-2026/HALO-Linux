# Building avoidance: planned changes (for Brooks)

Status: PLAN ONLY. Nothing below is implemented except the message file (see "Already in the working tree").
Author: Cohen, 2026-11-16. Reviewers: Brooks (Geolocation), Lisa (Movement).

## Hand-off

- Branch: `city-map`. Uncommitted: this file, `geolocation_msgs/msg/BuildingRisk.msg`, and one line in
  `geolocation_msgs/CMakeLists.txt`. No other code is written, so nothing needs reverting.
- Brooks: please answer "Needs Brooks's confirmation" (6 items) before anyone builds on the message.
  If the name, fields or scope change, edit or delete the `.msg` first; nothing consumes it yet.
- Lisa: please answer the three questions under "Movement".
- Then the plan below can be implemented as written (Geolocation, then Movement, then docs and scripts).

## Summary

Building avoidance follows the existing service boundary. Geolocation turns the drone's own position
uncertainty plus building data into an overlap probability. Movement turns that probability into a bounded
velocity bias. The only interface between them is one new topic.

## Needs Brooks's confirmation (not confirmed yet)

1. Message name `geolocation_msgs/BuildingRisk` and topic `/geolocation/building_risk`. AGENTS.md lists
   exact names for Geolocation as an open question. We proceed on these names and flag them in the PR.
2. Building risk lives in Geolocation as a second node (`building_risk_node`). Its only inputs are
   `/geolocation/position` and building data; it uses no perception or Tracking data.
3. `error_x_m` / `error_y_m` are ECEF axes, not local east/north. At Huntsville's latitude ECEF z carries part
   of the north error, so RMS(x, y) / 3 understates it. Acceptable for now, or rotate to ENU first?
4. Roof comparison: `altitude_m` is above the WGS84 ellipsoid. Proposal: the CSV gets a `base_alt_m` column
   (roof = base_alt_m + height_m). Vertical bound = `error_z_m`. OK?
5. `building_distance_m` is measured to the nearest disc edge (0 inside). Default "in range" radius is 200 m.
   OK?
6. DTED level and footprint layer (e.g. OSM) are undecided. Who supplies real footprints for Huntsville?

## Message: `geolocation_msgs/msg/BuildingRisk.msg`

| Field | Unit / frame |
| --- | --- |
| `std_msgs/Header header` | copied from the assessed `GeolocationState` |
| `float64 probability` | 0..1, chance the 3-sigma position uncertainty overlaps the riskiest nearby building |
| `float64 building_bearing_deg` | deg clockwise from true north, drone to that building, [0, 360) |
| `float64 building_distance_m` | m, horizontal, drone to nearest edge, 0 if inside |
| `float64 building_height_m` | m, roof height above the WGS84 ellipsoid |

No valid flag: nothing is published when no building is in range. `GeolocationState` does not change.

## Geolocation (Brooks)

New C++ node in the existing ament_cmake package (no language change).

- `Geolocation/include/geolocation/building_occlusion.hpp`, pure logic, no ROS calls:
  - `BuildingFootprint` (disc: id, lat, lon, radius_m, base_alt_m, height_m), `BuildingRiskEstimate`,
    `loadBuildingFootprints(path)` (skips bad rows, missing file gives empty list).
  - Horizontal sigma = RMS(error_x_m, error_y_m) / 3, isotropic Gaussian.
  - Overlap probability = closed-form circular Gaussian `exp(-c^2 / 2 sigma^2)`, c = clearance to the disc edge;
    1.0 inside the disc.
  - A building is excluded if the drone clears its roof even at the lowest 3-sigma altitude bound.
  - Only the loader and the overlap test change when DTED + polygons replace the discs.
- `Geolocation/include/geolocation/building_risk_node.hpp`, `src/building_risk_main.cpp`: node `building_risk`.
  Params: `input_topic`, `output_topic`, `buildings_file`, `max_range_m`. Reliable, depth 10, header copied,
  one startup log line, missing file gives a throttled warning and no output.
- `Geolocation/config/buildings_huntsville.csv`: placeholder discs, about one square mile around the
  courthouse square. `Geolocation/config/buildings_default_test.csv`: 3 discs near PX4's default Gazebo home
  for the default-world SITL test.
- `CMakeLists.txt`, `package.xml`, `launch/geolocation.launch.py` (starts the second node), `README.md`.
- Tests: `test/test_building_occlusion.cpp` (about 20 logic cases), `test/test_building_risk_node.cpp`
  (7 node cases: header copied, values, nothing published when none in range, missing file, non-finite input,
  parameter defaults).

## Movement (Lisa)

- `Movement/include/movement/building_avoidance.hpp`, pure logic. Defaults: `building_risk_threshold` 0.1,
  `building_risk_timeout_ms` 1000, `max_avoidance_m_s` 0.5, hard ceiling 1.5.
  1. Below threshold, stale/missing reading, or no heading: command unchanged.
  2. Otherwise repulsion away from the building bearing, speed = min(probability, 1) * max_avoidance_m_s.
  3. The sum is re-clamped to `max_forward_m_s` / `max_strafe_m_s`, then goes through the existing
     acceleration/jerk limiter. Yaw is never touched.
- `movement_node.hpp/.cpp`: subscribe `/geolocation/building_risk`, apply the blend in `tick()` before
  `limiter_.apply` for follow, orbit and search. Link-lost, search give-up Hold and `shutdown()` unchanged.
- Tests: `test/test_building_avoidance.cpp` (about 16 logic cases) and `test/test_node.cpp` additions
  (about 15 node cases, including stale, link lost, shutdown).
- `CMakeLists.txt`, `README.md`.

Questions for Lisa:
- In orbit mode, re-clamp strafe to the orbit speed cap instead of `max_strafe_m_s` (0.45 m/s)? Otherwise orbit
  slows sharply while avoidance is active. Proposal: use the orbit cap.
- The 1.5 m/s ceiling equals `kStrafeSpeedCeilingMps` (1.5), so it is not below it. Keep 1.5 or use 1.0?
- Is `max_avoidance_m_s: 0` enough of an off switch?

## Repo-wide

- `AGENTS.md`: service table, interfaces table row, data-flow edge Geolocation to Movement, design decision
  (building data = DTED + footprint layer, level/source TBD), new open questions (full covariance, waypoint
  replan, tuning against a real flight trace), note that the names are unconfirmed.
- `scripts/sim_check.py`, `scripts/baylands_mission.py`, `scripts/sim_stack`: add `building_risk_node` to the
  leftover-process patterns, because the launch file now starts two nodes. Root `README.md`: one line.
- CI: no change. The path filters already cover `geolocation_msgs/**`.

## Testing plan

1. Unit tests (no sim): colcon build and test for `geolocation_msgs`, `geolocation`, `movement`.
2. Safety review of the Movement diff against AGENTS.md "Safety and hardware readiness".
3. SITL in the default Gazebo world (empty, PX4 default home) with a few synthetic box buildings matching
   `buildings_default_test.csv`, run by the sim-operator only. Not Huntsville data, because the default world
   has no buildings and is not at Huntsville's coordinates. A Huntsville-world run comes later.

## Already in the working tree

- `geolocation_msgs/msg/BuildingRisk.msg` (new) and its entry in `geolocation_msgs/CMakeLists.txt`.
  Built with colcon. Nothing else is written.
- Container note: colcon builds in this container need
  `PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages` (the CMake template step cannot import `ament_package`
  otherwise).

## Next steps

1. Brooks answers the questions above; Lisa answers hers.
2. Implement Geolocation, then Movement, then AGENTS.md and script updates.
3. Run the unit tests, the safety review, and the default-world SITL check.
4. Later: replace the disc CSV with DTED + a footprint layer (loader and overlap test only).
