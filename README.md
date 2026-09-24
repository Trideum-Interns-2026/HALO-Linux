HALO
Repo for the HALO drone project

Officers often face dangerous situations where human lives are at risk. To assist the police department, we will use a drone to allow officers to monitor areas that could be dangerous for a person. Our goal is to design and test a drone system within budget, capable of safely assisting the Police Department with target pursuit operations and surveillance through QGroundControl simulation algorithms.

keane test commit
yo im will
Cohen updated the read-me
Brooks updated the read-me



# Running C++ programs
From the repository root, run:

```powershell
python run_all.py <folder> <file.cpp>
```

The folder search is recursive, so this runs the file inside the Geolocation subfolder:

```powershell
python run_all.py Tracking_Geolocation takeoff_forward_back.cpp
```

For the Movement folder:

```powershell
python run_all.py Movement test_takeoff.cpp
```

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

The sim stack is [px4io/px4-sitl-gazebo-ros2](https://hub.docker.com/r/px4io/px4-sitl-gazebo-ros2)
(PX4 SITL + Gazebo Harmonic + ROS 2 Jazzy + Micro XRCE-DDS Agent), built via
the repo's `Dockerfile`/`docker-compose.yml`. There is no other stack —
build from source and Gazebo Classic are gone.

1. `docker compose up` — builds the image (first run) and starts the
   container, which launches the Micro XRCE-DDS Agent.
2. In another terminal, `docker compose exec -e PX4_SIM_MODEL=gz_x500_depth px4-sitl /usr/local/bin/ros2-entrypoint.sh px4-gazebo`
   — starts PX4 SITL + Gazebo Harmonic with the `gz_x500_depth` vehicle (the
   X500 with an OakD-Lite depth camera; the plain `gz_x500` has no camera).
3. `ros2 launch perception gz_harmonic_bridge.launch.py` — bridges the
   camera topics from Gazebo into ROS 2 (see the topic-name caveat above —
   check `gz topic -l` first if nothing shows up).
4. `ros2 launch perception perception.launch.py` — runs the red-sphere
   detector against the bridged camera feed.

`Perception/scripts/spawn_sphere.sh` and `run_perception_demo.sh` (the old
one-shot "spawn a sphere and watch detection" workflow) are **stale** — they
call `gazebo_ros`'s `spawn_entity` service, which doesn't exist in Gazebo
Harmonic. They need porting to Harmonic's entity-spawn service before they'll
work again.

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
