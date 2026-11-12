# HALO

HALO is a police-support drone, developed in simulation: PX4 SITL + Gazebo Harmonic +
ROS 2 Jazzy, controlled over MAVSDK/MAVLink and monitored in QGroundControl.

Officers often face dangerous situations where human lives are at risk. HALO lets officers
monitor and pursue targets in areas that could be dangerous for a person, with a drone system
designed and tested within budget.

The system is split into microservices, each with its own owner, interface and tests.
**Read [AGENTS.md](AGENTS.md) for the architecture, service table, rules and data flow.**

| Service | Job | Code |
| --- | --- | --- |
| Perception | Find a person in camera images, publish `perception_msgs/Detection` | [Perception/](Perception/README.md), `perception_msgs/` |
| Geolocation | Estimate the drone's own position | [`Geolocation/`](Geolocation/README.md), [`geolocation_msgs/`](geolocation_msgs/) |
| Tracking | Estimate the person's position | [`Tracking/`](Tracking/README.md), [`tracking_msgs/`](tracking_msgs/) |
| Movement | Follow the tracked person, commanded over MAVLink | [`Movement/`](Movement/README.md) |
| UI/UX | Web view of the drone's data through a ROS bridge node | [`UI/`](UI/README.md) (bridge node), [`GroundstationAPI/`](GroundstationAPI/README.md) (web API + dashboard) |

## Common commands

Run inside the dev container. Each command is described in its service README.

```bash
fly                          # PX4 SITL + Gazebo (HEADLESS=1 fly for no window); leave it running
simlinks                     # sim only: GPS bridge and uXRCE-DDS agent; run next to fly
perceive                     # camera bridge + YOLO detector (Spark profile, needs Triton)
geolocation                  # drone position service
tracking                     # person position service
ui                           # UI bridge node (posts to the web API)
movement                     # follow service; needs an armed, airborne vehicle
spawn_person [x] [y] [yaw]   # sim only: test target, default 5 0 0 (5 m ahead, facing the drone)
simstack up [--gui]          # sim only: whole sim stack in one command, left running
simstack status              # which parts of the stack are running
simstack down                # stop what simstack started
groundstation                # web dashboard at http://127.0.0.1:8000 (API docs at /docs)
halo_test                    # all tests (no sim, GPU or Triton needed)
sim_check [--gui|--watch]    # sim only: whole stack, flies it, PASS/FAIL table, tears down
sitl_vehicle arm|takeoff [m]|land|disarm|status|position|link   # sim only, run by a human
```

`fly`, `simlinks`, `perceive`, `geolocation`, `tracking`, `ui`, `movement` and `./scripts/move` do not
arm or take off. Arm and take off in QGroundControl, or with `sitl_vehicle` in the sim.

## Prerequisites

Docker, WSL2 (Ubuntu), QGroundControl. Code runs inside a dev container
(ROS 2 Jazzy, PX4 SITL, Gazebo Harmonic); the repo is mounted at `/workspace`.

- Windows / WSL2: [Windows-Setup-Repo-to-Gazebo.md](Windows-Setup-Repo-to-Gazebo.md),
  and [PX4-SITL-Docker-Troubleshooting.md](PX4-SITL-Docker-Troubleshooting.md) if the sim misbehaves.
- DGX Spark (ARM64, shared host, required for anything that calls Triton):
  [docs/spark/README.md](docs/spark/README.md), including QGroundControl over TCP
  ([section 7](docs/spark/README.md#7-developer-qgroundcontrol-on-your-laptop)).

## Running the simulation

Either bring the whole stack up in one command, or start the parts yourself.

With one command (never arms or flies the vehicle):

```bash
simstack up --gui            # or: simstack up (headless); simstack down to stop
```

By hand:

1. Open QGroundControl.
2. Terminal 1: `fly` starts PX4 SITL and Gazebo (`HEADLESS=1 fly` for no window). Leave it running.
3. Terminal 2: `perceive` starts the camera bridge and the YOLO detector (Spark profile only,
   it needs Triton). Detections appear on `/perception/detection`.
4. Optional: `spawn_person [x] [y] [yaw]` adds a test target (default 5 m ahead, facing the drone).
5. Arm and take off in QGroundControl.
6. Start `simlinks` (GPS bridge), then `geolocation` to filter `/gps/fix`, then `tracking` to combine
   detections, camera calibration, Geolocation output and PX4 attitude.
7. Optional: `ui` (and `groundstation` for the dashboard) to see the positions in a browser.

Check the output with `ros2 topic echo /perception/detection`.
Geolocation publishes `/geolocation/position`; Tracking publishes `/tracking/position`.

### Movement

`movement` is the Movement service (see [Movement/README.md](Movement/README.md)). It follows
`/tracking/position` and needs `simlinks`, `geolocation` and `tracking` running, and a vehicle that is
armed and airborne. The waypoint test is also available:

```bash
./scripts/move <north_m> <east_m>   # short waypoint test, each offset <= 2 m, vehicle must be airborne
```

`move` holds altitude, never arms, takes off or lands, and returns PX4 to Hold when it finishes,
times out or is interrupted.

## Building and testing

```bash
# ROS 2 packages (workspace is /opt/ros2_ws; never run colcon in /workspace)
source /opt/ros/jazzy/setup.bash
source /opt/px4_ros2/install/setup.bash
cd /opt/ros2_ws && colcon build \
  --packages-select perception_msgs perception geolocation_msgs geolocation \
  tracking_msgs tracking movement ui --symlink-install
source /opt/ros2_ws/install/setup.bash

# All tests (no sim, GPU or Triton needed)
halo_test

# Per service
cd /opt/ros2_ws && colcon test --packages-select geolocation_msgs geolocation tracking_msgs tracking movement
colcon test-result --verbose
cd /workspace/Perception && python3 -m pytest test/ -v

# C++ waypoint test in Src/ (plain CMake, not colcon)
cmake -S /workspace -B /workspace/build
cmake --build /workspace/build --target move
```

CI runs each service's tests on push: `.github/workflows/`.

## Repo layout

| Path | What it is |
| --- | --- |
| `Perception/`, `perception_msgs/` | Perception service and its message package |
| `Geolocation/`, `geolocation_msgs/` | Drone GPS filtering service and output interface |
| `Tracking/`, `tracking_msgs/` | Person-position service and output interface |
| `Movement/` | Movement service (C++) |
| `UI/` | UI bridge node (ROS to HTTP) |
| `GroundstationAPI/` | Web API and dashboard (FastAPI) |
| `Src/` | C++ waypoint test (`move.cpp`) |
| `scripts/` | `fly`, `sim_stack`, `sim_links`, `sim_check`, `sitl_vehicle`, `halo_test`, `move`, `spark/verify.sh` |
| `infra/triton/` | Shared Triton Inference Server config and engine build script |
| `Worlds/huntsville/` | Custom Gazebo world (downtown Huntsville) |
| `.devcontainer/`, `Dockerfile`, `docker-compose*.yml` | Dev container profiles (WSL2 and Spark) |
| `docs/spark/` | DGX Spark setup guide |
| `.github/workflows/` | CI, one workflow per service |

## Docker resources

The Compose files set no CPU or storage limits. If Docker reports zero CPUs or the container exits
from resource pressure, check Docker Desktop's WSL2 allocation and free disk space. Read-only checks:
`docker info`, `docker system df`, `free -h`, `df -h`. Avoid pruning Docker data unless you have
confirmed it is safe.
