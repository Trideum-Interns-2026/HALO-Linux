# Building avoidance: planned changes (for Brooks)

Status: Geolocation portion implemented; Movement and repo-wide integration remain.
Author: Cohen, 2026-11-16. Reviewers: Brooks (Geolocation), Lisa (Movement).

## Hand-off

- Brooks's decisions are recorded below. The Geolocation node, pure logic, placeholder CSVs, tests,
  launch integration, and README update are implemented and pass package tests.
- Lisa: please answer the three questions under "Movement" before implementing Movement.
- Movement and repo-wide documentation/script integration remain to be implemented.

## Summary

Building avoidance follows the existing service boundary. Geolocation turns the drone's own position
uncertainty plus building data into an overlap probability. Movement turns that probability into a bounded
velocity bias. The only interface between them is one new topic.

## Brooks's decisions

1. Use message `geolocation_msgs/BuildingRisk` and topic `/geolocation/building_risk`.
2. Keep building-risk estimation in a second Geolocation node. It uses only Geolocation position and
   building data, not perception or Tracking data.
3. Rotate the per-axis ECEF bounds into local ENU before deriving horizontal sigma. The estimate
   assumes independent ECEF axes because the message does not carry covariance terms.
4. Compare the lowest 3-sigma altitude (`altitude_m - error_z_m`) with roof altitude
   (`base_alt_m + height_m`), both referenced to the WGS84 ellipsoid.
5. Measure horizontal distance to the nearest disc edge (zero inside), with a 200 m default range.
6. Real Huntsville data is deferred. The checked-in CSVs are explicitly synthetic placeholders;
   DTED level, authoritative footprint source, and data supplier must be decided before using real
   building data.

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
  - Rotate the independent ECEF 3-sigma bounds into ENU, then use the RMS of east/north standard
    deviations as the isotropic horizontal Gaussian sigma.
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
- Tests: `test/test_building_occlusion.cpp` (13 test cases), `test/test_building_risk_node.cpp`
  (5 test cases covering header/values, no nearby building, missing file, invalid input, and defaults).

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

- `geolocation_msgs/msg/BuildingRisk.msg` and its entry in `geolocation_msgs/CMakeLists.txt`.
- Geolocation node and tests are implemented; Movement remains pending Lisa's decisions.
- Container note: colcon builds in this container need
  `PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages` (the CMake template step cannot import `ament_package`
  otherwise).

## Next steps

1. Lisa answers the Movement questions.
2. Implement Movement, then update AGENTS.md and affected scripts/docs.
3. Run all unit tests, the Movement safety review, and the default-world SITL check.
4. Later: decide real terrain/footprint sources and replace the disc CSV through the loader and
   overlap-test boundary.
