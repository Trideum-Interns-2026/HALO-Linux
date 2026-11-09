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
| Geolocation | Estimate the drone's own position | C++ prototype in `Geolocation/` |
| Tracking | Estimate the person's position | C++ prototype in `Tracking/` |
| Movement | Follow the person, commanded over MAVLink | Older C++ programs in `Src/` |
| UI/UX | Web view of the drone's data through a ROS bridge node | Not started |

## Prerequisites

Docker, WSL2 (Ubuntu), QGroundControl. Code runs inside a dev container
(ROS 2 Jazzy, PX4 SITL, Gazebo Harmonic); the repo is mounted at `/workspace`.

- Windows / WSL2: [Windows-Setup-Repo-to-Gazebo.md](Windows-Setup-Repo-to-Gazebo.md),
  and [PX4-SITL-Docker-Troubleshooting.md](PX4-SITL-Docker-Troubleshooting.md) if the sim misbehaves.
- DGX Spark (ARM64, shared host, required for anything that calls Triton):
  [docs/spark/README.md](docs/spark/README.md), including QGroundControl over TCP
  ([section 7](docs/spark/README.md#7-developer-qgroundcontrol-on-your-laptop)).

## Running the simulation

1. Open QGroundControl.
2. Terminal 1: `fly` starts PX4 SITL and Gazebo (`HEADLESS=1 fly` for no window). Leave it running.
3. Terminal 2: `perceive` starts the camera bridge and the YOLO detector (Spark profile only,
   it needs Triton). Detections appear on `/perception/detection`.
4. Optional: `spawn_person [x] [y] [yaw]` adds a test target (default 5 m ahead, facing the drone).
5. Arm and take off in QGroundControl. `fly`, `tracking`, `geolocation` and `move` do not arm or
   take off.

Check the output with `ros2 topic echo /perception/detection`.

### Older tracking and movement programs

These programs are being replaced by the Tracking and Movement services and are partly out of date
(see "Known broken or stale" in [AGENTS.md](AGENTS.md)). They are for SITL only, never a real vehicle.

```bash
./scripts/move <north_m> <east_m>   # short waypoint test, each offset <= 2 m, vehicle must be airborne
```

`move` holds altitude, never arms, takes off or lands, and returns PX4 to Hold when it finishes,
times out or is interrupted. The old `follow_person` path still expects perception topics that the
current Perception service no longer publishes.

## Building and testing

```bash
# ROS 2 Perception packages (workspace is /opt/ros2_ws; never run colcon in /workspace)
cd /opt/ros2_ws && colcon build --packages-select perception_msgs perception --symlink-install
source install/setup.bash

# Perception tests, no sim, GPU or Triton needed
cd /workspace/Perception && python3 -m pytest test/ -v

# C++ programs and the Geolocation unit test
cmake -S /workspace -B /workspace/build
cmake --build /workspace/build --target geolocation tracking move follow_person drone_gps_kalman_filter_test
ctest --test-dir /workspace/build --output-on-failure
```

CI runs each service's tests on push: `.github/workflows/`.

## Repo layout

| Path | What it is |
| --- | --- |
| `Perception/`, `perception_msgs/` | Perception service and its message package |
| `Geolocation/`, `Tracking/` | C++ drone and target position prototypes |
| `Src/` | Older C++ movement and follow programs |
| `scripts/` | `fly`, `move`, `follow_person`, `spark/verify.sh` |
| `infra/triton/` | Shared Triton Inference Server config and engine build script |
| `Worlds/huntsville/` | Custom Gazebo world (downtown Huntsville) |
| `.devcontainer/`, `Dockerfile`, `docker-compose*.yml` | Dev container profiles (WSL2 and Spark) |
| `docs/spark/` | DGX Spark setup guide |

## Docker resources

The Compose files set no CPU or storage limits. If Docker reports zero CPUs or the container exits
from resource pressure, check Docker Desktop's WSL2 allocation and free disk space. Read-only checks:
`docker info`, `docker system df`, `free -h`, `df -h`. Avoid pruning Docker data unless you have
confirmed it is safe.
