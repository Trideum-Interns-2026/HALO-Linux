HALO
Repo for the HALO drone project

Officers often face dangerous situations where human lives are at risk. To assist the police department, we will use a drone to allow officers to monitor areas that could be dangerous for a person. Our goal is to design and test a drone system within budget, capable of safely assisting the Police Department with target pursuit operations and surveillance through QGroundControl simulation algorithms.


# Prerequisites
Docker
WSL2
Ubuntu
ROS 2 foxy
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

## Running it

1. Open QGroundControl
2. `fly` — launches Gazebo.
3. In a new terminal, enter the command `perceive` — this runs perception code.
4. In a new terminal, enter the command `track` — runs the Kalman filter and repeatedly sends the filtered target
  location to QGroundControl as `MAV_CMD_DO_SET_ROI_LOCATION`, keeping the
  QGroundControl ROI command updated while the target is visible. The same
  terminal displays the drone and filtered target coordinates once per second.

Run `track` in a separate terminal after perception is running. The vehicle
must be connected on MAVSDK UDP `14540`. QGroundControl does not persistently
draw arbitrary map pins for ROI commands, so use the `track` terminal readout
for the live target latitude, longitude, altitude, and distance.

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
