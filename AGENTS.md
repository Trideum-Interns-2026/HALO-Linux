# AGENTS.md — HALO-Linux

Guide for AI coding agents working in this repo. Read it fully before changing anything.
MUST = required. NEVER = forbidden. SHOULD = default unless a comment explains why not.

## What this project is

HALO is a police-support drone, currently developed in simulation: PX4 SITL + Gazebo Harmonic +
ROS 2 Jazzy, controlled over MAVSDK/MAVLink, monitored in QGroundControl.

The system is split into microservices. They are equal peers: each has its own owner,
interface and tests, and no one service is the template or the priority. The rules in
"Microservice rules" apply to all of them the same way.

| Service | Owner | Job | Input | Output | Code today |
| --- | --- | --- | --- | --- | --- |
| Perception | Cohen | Find a person in camera images | camera image (ROS 2) | `perception_msgs/Detection`: header, confidence, bounding box in pixels | `Perception/`, `perception_msgs/` (done) |
| Geolocation | Brooks | Estimate the **drone's own** position | drone GPS (no perception data) | `geolocation_msgs/GeolocationState` (see "Position output format") | `Geolocation/`, `geolocation_msgs/` |
| Tracking (POI = person of interest) | Brooks | Estimate the **person's** position in the world | `Detection` (box + confidence), plus the drone's position | `tracking_msgs/TrackingState`, same format as Geolocation | `Tracking/`, `tracking_msgs/` |
| Movement | Lisa | Follow the person at a set distance, scan when lost | `tracking_msgs/TrackingState` (`/tracking/position`, person) + `geolocation_msgs/GeolocationState` (`/geolocation/position`, drone) + `camera_info` (ROS 2); drone heading from MAVSDK attitude (MAVLink) | vehicle motion only, sent over MAVLink as bounded offboard velocity commands (goto is still an open question) | `Movement/` (port of Lisa's `follow_person`), old `Src/move.cpp` waypoint test |
| UI/UX (web endpoint) | Kean | Show the operator the drone's data in a website on any device | `GeolocationState` + `TrackingState` via a ROS-side bridge node | HTTP to a website; QGroundControl socket; optional drone health | `UI/` (bridge node), `GroundstationAPI/` (FastAPI web API + dashboard) |

Design decisions already made:
- Every service output carries a timestamp (`std_msgs/Header`).
- Geolocation (drone) and Tracking (person) publish the same message format: a position triplet in
  LLA plus an error triplet in ECEF (see below). Only what they locate and their inputs differ.
- Range to the person is internal to Tracking, not a published output. The expected human height
  error is folded into the error triplet. That is why `Detection` has no distance field.
- Movement commands the vehicle only over MAVLink, NOT ROS, and owns following the person. Today it
  follows the Tracking position (Lisa's follow/orbit controller) with bounded offboard velocity
  commands; goto is still an open question (see "Open questions").
- The UI must NOT connect directly to ROS or the drone. A ROS-side bridge node translates data to
  web formats and serves it over the network (HTTP); MAVLink may be used for drone health.
- The old C++ tracking/follow code is gone (replaced by Tracking and Movement). `Src/move.cpp` is the
  one remaining old program; do not add features to it.

### Position output format (Geolocation and Tracking)

1. Position: latitude (deg), longitude (deg), altitude (m above the WGS84 ellipsoid, as PX4 GPS reports it).
2. Error: an x, y, z triplet in ECEF metres.
   Why not LLA for error: LLA is non-Euclidean, so an error volume in it warps and is hard to
   draw, and the error is radial. ECEF is Cartesian, so the error is a smooth sphere/ellipsoid
   that is easy to render (and suits radar).

Decided (see `geolocation_msgs/msg/GeolocationState.msg`, `tracking_msgs/msg/TrackingState.msg`):
fields `latitude_deg`, `longitude_deg`, `altitude_m`, and `error_x_m`/`error_y_m`/`error_z_m`, where the
error is a per-axis 3-sigma bound in ECEF metres. Ask Brooks before changing them; Movement and the UI
bridge consume them.

### Language choice

Each owner picks, following the general rule: C++ for performance-critical back-end components
(control loops, MAVLink/MAVSDK, high-rate filtering), Python for rapid prototyping, high-level
autonomy, and AI/ML orchestration (e.g. Perception calling Triton). Write the reason in the
service README if you go against it.

### Package names

Follow the existing pattern: folder `<Name>/` with package `<name>`, plus `<name>_msgs/` for its
types (e.g. `Geolocation/` + `geolocation_msgs/`, `Movement/`, `UI/`). Confirm the exact name with
the owner before creating a package.

## Repo map

| Path | What it is |
| --- | --- |
| `Src/` | `move.cpp` (waypoint test), the only old C++ program left |
| `Geolocation/`, `geolocation_msgs/` | Geolocation service + interface package |
| `Tracking/`, `tracking_msgs/` | Tracking service + interface package |
| `CMakeLists.txt`, `package.xml` (root) | Build for the C++ code above (plain CMake, built by `scripts/*`, NOT colcon) |
| `Perception/`, `perception_msgs/` | Perception service: node package + interface package |
| `UI/`, `GroundstationAPI/` | UI service: `ui` bridge node (ROS to HTTP, Python) + FastAPI web endpoint and dashboard (plain pip, tested in `ui-tests.yml`) |
| `Movement/` | Movement service (C++, ament_cmake): follow control math, MAVSDK adapter, node, tests |
| `scripts/` | `fly` (start sim), `sim_links`, `sim_stack` (whole stack up/down), `sim_check`, `sitl_vehicle`, `halo_test` (all tests), `move`, `spark/verify.sh`, `tests/` |
| `infra/triton/` | Shared Triton Inference Server: compose file, `build_engine.sh`, `model_repository/yolo_person/config.pbtxt` |
| `Worlds/huntsville/` | Custom Gazebo world (downtown Huntsville) |
| `.devcontainer/` | Dev container profiles: default (WSL2) and `spark/` (DGX Spark) + `post-create.sh` |
| `Dockerfile`, `docker-compose*.yml` | Base image (`px4io/px4-sitl-gazebo-ros2`) and compose overrides (`.wsl`, `.spark`) |
| `docs/spark/README.md` | Full Spark setup + QGroundControl over TCP |
| `.github/workflows/` | CI (`scripts-tests.yml`, `perception-tests.yml`, `geolocation-tests.yml`, `tracking-tests.yml`, `movement-tests.yml`, `ui-tests.yml`) |

## Environments

Two dev container profiles. Code runs inside the container; the repo is mounted at `/workspace`.

| | WSL2 (default) | DGX Spark |
| --- | --- | --- |
| Profile | `.devcontainer/devcontainer.json` | `.devcontainer/spark/devcontainer.json` |
| Arch | x86_64 | ARM64, shared headless host, one container per developer (`HALO_DEV` in `.env`) |
| GUI | WSLg | Selkies in the browser (port 8080), GL via `vglrun` |
| QGroundControl | UDP | On the laptop, TCP `127.0.0.1:5760` via a MAVLink TCP bridge |
| Triton / `tritonclient` | Not available | `HALO_TRITON_URL=triton:8001`, `tritonclient` installed by `.devcontainer/spark/Dockerfile` |

Facts that cause bugs if forgotten:
- Anything that calls Triton (today: perception) only runs on the Spark profile as of now.
- `PX4_SIM_MODEL=gz_x500_depth` (X500 + OakD-Lite). Plain `gz_x500` has no camera.
- The color camera is 1920x1080 at 30 Hz (the model default; the Dockerfile no longer caps it to 15).
- Spark containers are fenced to localhost (`ROS_LOCALHOST_ONLY=1`, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, `GZ_IP=127.0.0.1`) so developers don't see each other's sims. NEVER remove these.
- Gazebo world name is usually `default`; scripts find it dynamically via `Perception/scripts/gz_world.sh`.
- The camera bridge launch (`Perception/launch/gz_harmonic_bridge.launch.py`) and the person spawner live in `Perception/` today but are shared sim tooling; any service can use them.
- `ROS_LOCALHOST_ONLY is deprecated` warnings at node startup are expected; ignore them.

## Commands

In an agent session only the `sim-operator` agent runs sim commands (`fly`, `simlinks`, service
launches, `simstack`, `sim_check`, `sitl_vehicle`, `spawn_person`); the guard hook blocks everyone
else. The main agent hands it a command from the approved list in `.claude/agents/sim-operator.md`.
Humans run them directly.

```bash
fly                       # terminal 1: PX4 SITL + Gazebo (waits until /world/default/create exists)
HEADLESS=1 fly            # same, no Gazebo window
perceive                  # terminal 2: camera bridge + YOLO detector (ros2 launch perception perception.launch.py)
movement                  # movement service (ros2 launch movement movement.launch.py); needs an armed, airborne vehicle
sim_check [--gui|--watch]   # one command: whole stack in the sim, arms and flies it, PASS/FAIL table, tears down (SITL only)
simlinks                  # sim-only: GPS bridge -> /gps/fix + uXRCE-DDS agent (/fmu/...); run next to fly
sitl_vehicle arm|takeoff [m]|land|disarm|status|position|link   # SITL only; human (/arm) or sim-operator; uses PX4 port 14580, runs alongside movement
spawn_person [x] [y] [yaw]   # test target, default 5 0 0 (5 m ahead, facing the drone)
tracking                  # tracking service (ros2 launch tracking tracking.launch.py)
geolocation               # geolocation service (ros2 launch geolocation geolocation.launch.py)
ui                        # UI bridge node (ros2 launch ui ui.launch.py)
groundstation             # web API + dashboard on http://127.0.0.1:8000 (port 8000 is forwarded by both dev containers)
simstack up [--gui]       # whole sim stack in one command, left running; never arms. simstack status | down
halo_test [scripts|perception|ui|web|ros]   # run every test suite, one PASS/FAIL line each
./scripts/move <n_m> <e_m>   # short waypoint test, |offset| <= 2 m, vehicle must already be airborne

# ROS 2 packages (build workspace is /opt/ros2_ws, packages symlinked from /workspace)
cd /opt/ros2_ws && colcon build --packages-select <pkg>_msgs <pkg> --symlink-install && source install/setup.bash

# C++ in the root project (Src/)
cmake -S /workspace -B /workspace/build && cmake --build /workspace/build --target move

# tests for any ROS 2 package
cd /opt/ros2_ws && colcon test --packages-select <pkg>_msgs <pkg> && colcon test-result --verbose
cd /workspace/<Pkg> && python3 -m pytest test/ -v        # Python packages, faster

# useful checks
ros2 topic list
ros2 topic echo <topic>
ros2 interface show <pkg>_msgs/msg/<Type>
gz topic -l | grep image
```

- NEVER run `colcon build` in `/workspace`: the root `package.xml` makes colcon see the wrong package.
- Python edits in an ament_python package need no rebuild (`--symlink-install`); restart the node. New files, `setup.py`, `package.xml`, `CMakeLists.txt`, `.msg` or any C++ change need a rebuild.
- `fly`, `tracking`, `geolocation`, `move` and `movement` do NOT arm or take off. Arm and take off in QGroundControl first.
- MAVSDK connects on `udpin://0.0.0.0:14540`. Ground-station MAVLink on UDP 14550.

## Data flow between services

```
camera (sim/HW) ──ROS 2──> Perception ──Detection──────────────┐
drone GPS ───────────────> Geolocation ──drone LLA + error──┬──> Tracking ──person LLA + error──┬──> Movement ──MAVLink offboard velocity──> vehicle
                                                             │                                   └──> UI bridge node ──HTTP──> website
                                                             ├──────────────────────────────────────> UI bridge node
                                                             └──────────────────────────────────────> Movement
vehicle telemetry (MAVLink) ──> QGroundControl, UI bridge node (drone health)
```

Transport rules:
- Between ROS services: ROS 2 topics with typed messages from `_msgs` packages.
- To the vehicle: MAVLink (Movement only).
- To the UI: through a translation layer to web formats over HTTP; never a direct ROS or drone link.
- NEVER import another service's Python module or C++ header to share logic; share types through `_msgs` packages.

Interfaces that exist today:

| Topic | Type | Publisher | Notes |
| --- | --- | --- | --- |
| `/camera/front/image_raw` | `sensor_msgs/Image` | camera bridge (sim) | 1920x1080, 30 Hz in sim |
| `/camera/front/camera_info` | `sensor_msgs/CameraInfo` | camera bridge (sim) | intrinsics |
| `/perception/detection` | `perception_msgs/Detection` | Perception | header, confidence, x_min/y_min/x_max/y_max in pixels; only when a target is seen |
| `/geolocation/position` | `geolocation_msgs/GeolocationState` | Geolocation | drone LLA + ECEF 3-sigma error |
| `/tracking/position` | `tracking_msgs/TrackingState` | Tracking | person LLA + ECEF 3-sigma error |
| HTTP `POST /api/telemetry` (not ROS) | JSON, `Telemetry` model in `GroundstationAPI/main.py` | `ui` bridge | drone LLA, person LLA when tracked; 2 Hz; read back by the dashboard from `GET /api/telemetry/latest` |

## Microservice rules (every service)

Each service is written in whichever language fits it: Python (`ament_python`) or C++
(`ament_cmake`, rclcpp). The rules are the same; language-specific notes are marked.

Layout for a service `<name>`:
```
<Name>/                  node package, package name lowercase <name>
  package.xml, README.md, launch/<name>.launch.py, test/
  Python: setup.py, setup.cfg, resource/<name> (empty, must match package name),
          <name>/__init__.py, <name>/<node>.py
  C++:    CMakeLists.txt, include/<name>/*.hpp, src/*.cpp
<name>_msgs/             ament_cmake interface package: package.xml, CMakeLists.txt, msg/<Type>.msg
                         (no src/ or include/)
.github/workflows/<name>-tests.yml
```

Messages:
- First field `std_msgs/Header header`; the node copies the input's header.
- `CamelCase.msg`, `snake_case` fields, a unit + frame comment on every field, SI units.
- Named fields over fixed-size arrays. No "valid" flags: don't publish when there is nothing.
- Changing a field breaks consumers: update them in the same PR.

Node code:
- One node = one job. A service may have more than one node if each has its own job.
- Logic in plain functions or classes with no ROS calls inside, so it is unit-testable without ROS (Python: `@staticmethod` / module functions; C++: free functions or a plain class in a header).
- Every tunable value is a ROS parameter with a default, read in the constructor. Env vars only as a parameter default.
- Fixed values as named constants (Python `UPPER_CASE`, C++ `constexpr kName`).
- Sensor inputs: sensor-data QoS (`qos_profile_sensor_data` / `rclcpp::SensorDataQoS()`). Outputs: reliable, depth 10, under `/<name>/`.
- External calls (Triton, MAVSDK, services) behind one method or interface so tests can replace it; on failure log with throttling and skip or fail safe, never crash.
- Output messages copy the input header when they describe that input.
- One startup log line with the effective config; no per-message info logs.
- Clean shutdown on Ctrl+C (Python: `try`/`except KeyboardInterrupt`/`finally: destroy_node(); shutdown()`; C++: stop timers/offboard, then `rclcpp::shutdown()`).
- Python: `float()`/`int()` numpy scalars before putting them in a message. Lines ≤ 99 chars.
- NEVER `sleep` or block in callbacks. NEVER load ML model weights in a node (use Triton).

Packaging and container:
- Every launch file is installed (Python: `setup.py` `data_files`; C++: `install(DIRECTORY launch ...)`).
- `package.xml`: real description and maintainer (not `root`/`TODO`), `depend` for ROS imports, `exec_depend` for runtime-only deps; pip-only deps get an XML comment.
- `.devcontainer/post-create.sh`: symlink both packages into `/opt/ros2_ws/src`, add them to `colcon build --packages-select`, add one alias for the launch file. Remove aliases for deleted scripts in the same change.
- Only GUI commands get `vglrun` in `spark/post-create.sh`.
- README in the node package: topics table, parameters table, run and test commands.

## AI inference (Triton) — for any service that runs a model

Today only Perception uses it; any service that adds a model follows the same rules (its own
model in `infra/triton/model_repository/<model>/`).

- Server: one shared `tritonserver` on the Spark (`infra/triton/docker-compose.yml`), reached at `triton:8001` (gRPC) on the `halo-inference` network. Run by the Spark admin, not per developer.
- Existing model `yolo_person`: TensorRT plan built ON the Spark by `infra/triton/build_engine.sh` (`.plan` files are GPU-specific and NEVER committed). `max_batch_size` 8, dynamic batching.
- I/O from `config.pbtxt`: input `images` FP32 `[3,640,640]`; output `output0` FP32 `[84,8400]` = 4 box values (cx, cy, w, h in 640 letterbox pixels) + 80 COCO class scores; person = class 0 = row 4.
- Preprocessing must match how the model was trained. For `yolo_person`: letterbox to 640x640 (scale = min(640/w, 640/h), centered, pad value 114), BGR→RGB, HWC→CHW, float32 / 255, add batch dim, `np.ascontiguousarray`.
- Postprocessing: subtract padding, divide by scale, clip to frame. Single best person = `argmax` of row 4 (no NMS needed). Multiple people = threshold + `cv2.dnn.NMSBoxes` (IoU 0.7).
- Always pass `client_timeout` to `infer`.
- Client and server must come from the same Triton release (`TRITON_TAG`, default 25.09). On ARM, `tritonclient` comes from the SDK image wheel, not plain pip.

## Tests and CI

- Tests MUST run with no sim, GPU or Triton.
- Two layers per service:
  - Logic tests: the pure functions, parametrized over input shapes, with edge cases (exact thresholds, limits/clipping, multiple candidates, nothing found, round trips). Python: pytest. C++: gtest via `ament_cmake_gtest`.
  - Node tests: a real node with the external call and publisher faked. Check header copied, values, nothing published when nothing found, failures don't crash, parameter defaults.
- Motion code (Movement) additionally tests that every limit is enforced and that loss of target or connection leads to a safe stop.
- CI: one workflow per service, `.github/workflows/<name>-tests.yml`, in a plain `ros:jazzy` container, filtered to that service's paths. `perception-tests.yml` is an existing example of the format.
- NEVER skip, delete or weaken a test to make CI pass.
- `Src/move.cpp` has no tests or CI.

## Safety and hardware readiness

Today everything runs in SITL, but the same code will later run against a flight controller in
HITL and on real hardware. Write it so that switch is a configuration change, not a rewrite.

- Connection endpoints (MAVSDK URL, MAVLink host/port), topic names, and camera parameters MUST be
  ROS parameters or CLI arguments with the SITL values as defaults. NEVER hardcode `udpin://0.0.0.0:14540`
  or `/world/default/...` inside logic.
- Sim-only helpers (spawn scripts, Gazebo bridges, `gz` calls, `/world/...` services) stay in
  `scripts/`, launch files, or a package's `scripts/`. NEVER call them from any service's node code.
- Read camera size and intrinsics from `camera_info` / the image itself, not from constants matching
  the Gazebo camera (1920x1080, 30 Hz).
- Use the message header stamp (or ROS time) for timing, never wall-clock assumptions about sim speed.
- Every motion command MUST be bounded: speed, yaw rate, offset, and follow-distance limits as named
  constants or parameters, enforced in code.
- Every offboard / motion program MUST stop and return PX4 to Hold on exit, Ctrl+C, timeout, lost
  target, or lost connection (see `OffboardGuard` in `move.cpp`; `Movement/` leaves offboard via `MovementNode::shutdown`).
  Exception, by Lisa's design: `Movement/` scans for a lost target (`search_timeout_s`, default 0 = until found) instead of stopping at once; set a timeout to hand back to Hold. It always leaves offboard on exit.
- NEVER arm, take off, or command motion automatically unless that program's documented purpose is
  to do so (today only `scripts/sitl_vehicle` and `scripts/sim_check`, run by a human or by the sim-operator in the sim, local endpoints only, takeoff <= 10 m). Programs assume a human armed the vehicle.
- An agent MUST NOT run anything against a real vehicle or flight controller, or change code paths
  meant for one, without explicit instruction from a human in the current task.

## Notes on `Src/move.cpp`

- C++17, MAVSDK + rclcpp + Eigen. Each executable is its own `add_executable` in the root `CMakeLists.txt`.
- Offboard control returns PX4 to Hold on exit, timeout or Ctrl+C (see `OffboardGuard` in `move.cpp`).

## Before finishing any change

- [ ] Builds: colcon for ROS packages, cmake for `Src/`
- [ ] Tests pass for every touched service
- [ ] Consumers of any changed topic or message are updated
- [ ] Aliases, scripts, launch files and READMEs that reference renamed or deleted files are updated
- [ ] No debug logs, hardcoded values, or unused generated files left behind
- [ ] Nothing new is hardcoded to SITL (endpoints, world names, camera size); motion stays bounded and fails safe

## Open questions

Ask the owner before deciding any of these in code.

General
- Should the WSL profile get `tritonclient` and a Triton endpoint, or are model-using services Spark-only?
- Exact package and topic names for Geolocation, Tracking, Movement and UI (pattern above).

Movement (Lisa)
- Datatype for movement commands (current idea: "functions").
- How the MAVLink goto command works (e.g. MAVSDK `goto_location` / `MAV_CMD_DO_REPOSITION`); needs research.
- Follow distance and circling radius around the POI.

UI/UX web endpoint (Kean)
- What the bridge node sends (which fields, how often) and HTTP vs WebSocket.
- Whether a database is needed.
