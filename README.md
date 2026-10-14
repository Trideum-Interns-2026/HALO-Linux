HALO
Repo for the HALO drone project

Officers often face dangerous situations where human lives are at risk. To assist the police department, we will use a drone to allow officers to monitor areas that could be dangerous for a person. Our goal is to design and test a drone system within budget, capable of safely assisting the Police Department with target pursuit operations and surveillance through QGroundControl simulation algorithms.


# Prerequisites
Docker
WSL2
Ubuntu
ROS 2 jazzy
QGroundControl

# Running C++ programs
From the repository root, run:

```powershell
python run_all.py <folder> <file.cpp>
```

For example:

```powershell
python run_all.py Src takeoff_forward_back.cpp
```
The search is recursive, so only the first folder is required.


# Perception (vision system)

`Perception/` is a ROS 2 (ament_python) package that detects a red sphere
from the drone's camera feed using OpenCV, and estimates distance to it.

- The camera feed comes from the `gz_x500_depth` vehicle's OakD-Lite depth
  camera, bridged from Gazebo Harmonic to ROS 2 via `ros_gz_bridge`
  (`Perception/launch/gz_harmonic_bridge.launch.py`, configured by
  `Perception/config/gz_harmonic_camera_bridge.yaml`) onto
  `/camera/front/image_raw` and `/camera/front/camera_info`. The Gazebo-side
  topic names in that config are a best guess from PX4's OakD-Lite model
  source and are **not confirmed** — after bringing the sim up, run
  `gz topic -l` inside the container and fix them if they don't match.
- `perception/red_sphere_detector.py` subscribes to that topic and to its
  matching `/camera/front/camera_info`, detects the largest red contour via
  HSV thresholding, and publishes:
  - pixel centroid/radius on `/red_sphere_detector/red_sphere/position`
  - a distance estimate in meters on `/red_sphere_detector/red_sphere/distance`
    (pinhole model: known sphere diameter × focal length ÷ apparent radius —
    only as accurate as the `sphere_diameter_m` parameter matches the actual
    target)
  - a bounding-box-annotated debug image on
    `/red_sphere_detector/red_sphere/debug_image`
  - the raw HSV threshold mask on `/red_sphere_detector/red_sphere/mask_debug`
    (all-black here means the color filter isn't seeing anything red —
    useful for debugging before assuming the detection logic is broken)
- `Perception/models/red_sphere.sdf` is a red test sphere (0.4m diameter) you
  can spawn into the running world.
- `perception/person_detector.py` detects a person with no marker, using
  OpenCV's built-in HOG people detector, and publishes on the same kinds of
  topics as the sphere detector:
  - pixel centroid/box height on `/person_detector/person/position`
  - a distance estimate on `/person_detector/person/distance` (from box
    height, assuming the person is `person_height_m` = 1.9m tall — matches
    the test model)
  - an annotated debug image on `/person_detector/person/debug_image`
- `Perception/models/standing_person.sdf` is a static standing human (mesh
  from Gazebo Fuel, downloaded to `~/.gz/fuel` on first spawn — needs
  internet once). `standing_person_marker.sdf` is the same person with a
  0.4m red sphere on the chest, for the red sphere detector. Spawn with
  `spawn_person [x] [y] [yaw]` (default 5 0 0, facing the drone), or
  `MARKER=1 spawn_person` for the marked one. Spawning it removes
  `red_sphere`, so the detectors only ever see one target.

To switch the automatic `fly` workflow from the sphere to a person, set
`PERCEIVE_TARGET` before running it:

- `PERCEIVE_TARGET=person` — unmarked person, `person_detector`
- `PERCEIVE_TARGET=marked_person` — person with red marker,
  `red_sphere_detector` (`track` needs no variable for this one)

## Running it

1. Open QGroundControl.
2. Run `fly` in the Dev Container. It starts PX4/Gazebo, spawns the selected
   test target, starts the camera bridge and detector without debug-image
   publishing, and builds/runs the target-centering controller.
3. The controller waits for PX4 health checks, arms, and takes off using PX4's
   configured `MIS_TAKEOFF_ALT` (2.5 meters by default in this SITL setup).
   It Kalman-filters the target's camera bearing and continuously yaws to keep
   the target centered. When detections go stale, it holds its current heading;
   press Ctrl+C to land and stop the simulator and ROS processes.

The default target is a red sphere. Select a person or a marked person by
setting `PERCEIVE_TARGET=person` or `PERCEIVE_TARGET=marked_person` before
running `fly`. The existing `perceive` command remains available when you want
the standalone perception demo and debug-image viewer.

Automatic takeoff and tracking are intended for PX4/Gazebo SITL only; do not
run them against a real vehicle. QGroundControl and the vehicle must be
connected through MAVLink, and the controller uses MAVSDK UDP `14540`.

### Docker resources

The Compose files do not set CPU or storage limits. `fly` runs inside the
already-created Dev Container; it cannot allocate more CPU or disk to Docker.
If Docker reports zero CPUs or the container exits from resource pressure,
check Docker Desktop's WSL2 resource allocation and available host/WSL disk
space before rerunning. Read-only checks include `docker info`,
`docker system df`, `free -h`, and `df -h`; avoid pruning Docker data unless
you have confirmed it is safe to remove.

## Running on the NVIDIA DGX Spark (ARM64)

The base image and this repo's `Dockerfile` are multi-arch (amd64 + arm64),
so the same `docker compose build` produces a native arm64 image on the
Spark — no `--platform` flag needed, and the MAVSDK install step picks the
matching arm64 release asset automatically.

The only thing that differs on the Spark is GPU/display forwarding, which
`docker-compose.spark.yml` handles: real GPU passthrough via the NVIDIA
Container Toolkit (Compose's device-reservation equivalent of
`docker run --gpus`) plus plain X11, instead of the Windows/WSL2-only
WSLg/D3D12 setup in `docker-compose.wsl.yml`.

One-time setup on the Spark:

1. Install the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
   and configure it for Docker: `sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker`.
   Confirm it's picked up with `docker info | grep -i nvidia` (should list
   `nvidia` under Runtimes).
2. Allow Docker to connect to the local X server: `xhost +local:docker`.

Then, from this folder on the Spark:

```bash
docker compose -f docker-compose.yml -f docker-compose.spark.yml build
docker compose -f docker-compose.yml -f docker-compose.spark.yml run --rm px4-sitl
```

From there, the workflow is the same as the "Running it" section above.
