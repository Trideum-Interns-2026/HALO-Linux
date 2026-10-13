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

`Perception/` is a ROS 2 (ament_python) package that finds a person in the
drone's camera feed with YOLOv8n and estimates how far away they are. That's
what the demo runs; `perceive` and `track` use it by default.

- The camera feed comes from the `gz_x500_depth` vehicle's OakD-Lite color
  camera (1920x1080, capped at 15 Hz by the Dockerfile), bridged from Gazebo
  Harmonic to ROS 2 via `ros_gz_bridge`
  (`Perception/launch/gz_harmonic_bridge.launch.py`) onto
  `/camera/front/image_raw` and `/camera/front/camera_info`. The bridge finds
  the camera's Gazebo topic in whatever world is running, and the spawn
  scripts likewise target whichever world is running, so nothing here is
  tied to PX4's `default` world. If a world has more than one camera, pick
  one with `ros2 launch perception gz_harmonic_bridge.launch.py
  gz_image_topic:=<gz topic>`; export `WORLD=<name>` to override the spawn
  scripts' world.
- `perception/yolo_person_detector.py` runs YOLOv8n on the NVIDIA GPU (CPU if
  PyTorch can't see one), keeps the most confident person, and publishes:
  - pixel centroid and box height on `/yolo_person_detector/person/position`
    (PointStamped: x=u, y=v, z=box height in px)
  - a distance estimate in meters on `/yolo_person_detector/person/distance`
    (pinhole model: `person_height_m` × focal length ÷ box height, with
    `person_height_m` = 1.9m to match the test model)
  - an annotated debug image (960x540, box + confidence + distance) on
    `/yolo_person_detector/person/debug_image`, only drawn while something
    is watching it

  PyTorch, Ultralytics, and the model weights (`/opt/yolo/yolov8n.pt`) are
  installed by the Dockerfile. Inference takes ~15 ms per frame on the GPU,
  so it keeps up with the camera; if it falls behind, it skips to the newest
  frame rather than building a backlog.
- `Perception/models/standing_person.sdf` is a static standing human (mesh
  from Gazebo Fuel, downloaded to `~/.gz/fuel` on first spawn — needs
  internet once). `perceive` spawns it for you; to move it, run
  `spawn_person [x] [y] [yaw]` (default 5 0 0, facing the drone).
- `perception/debug_image_viewer.py` is the live viewer `perceive` opens on
  the debug image. Use it instead of `rqt_image_view`: rqt subscribes
  best-effort, which drops most of these large frames with this DDS setup
  (measured ~0.6 Hz vs ~15 Hz with this viewer).

## Running it

1. Open QGroundControl
2. `fly` — launches PX4 SITL and Gazebo.
3. In a new terminal, `perceive` — spawns the person, starts the camera
   bridge and the YOLO detector, and opens the viewer window with the
   bounding box. Close the window (or press q / Esc) to stop all of it.
4. In a new terminal, `track` — runs the Kalman filter on the YOLO
   detections and repeatedly sends the filtered target location to
   QGroundControl as `MAV_CMD_DO_SET_ROI_LOCATION`, keeping the ROI command
   updated while the person is visible. The same terminal displays the drone
   and filtered target coordinates once per second.

The vehicle must be connected on MAVSDK UDP `14540`. QGroundControl does not
persistently draw arbitrary map pins for ROI commands, so use the `track`
terminal readout for the live target latitude, longitude, altitude, and
distance.

If the view looks laggy, the `perceive` terminal says which stage is slow.
The detector logs a line every 5 s (`camera in … Hz, processed … Hz, dropped
… stale | inference avg … ms`) and the viewer logs `received … Hz, displayed
… Hz`; all of them should be close to 15 Hz. The first detection after
startup takes a few seconds while CUDA warms up.

### Other detectors (not used for the demo)

Earlier detectors are still in the package, selected by setting
`PERCEIVE_TARGET` in **both** the `perceive` and `track` terminals:

- `PERCEIVE_TARGET=person` — same person, OpenCV HOG (`person_detector.py`);
  less robust to distance, pose, and angle than YOLO
- `PERCEIVE_TARGET=marked_person` — person with a 0.4m red sphere on the
  chest, tracked by color (`red_sphere_detector.py`)
- `PERCEIVE_TARGET=sphere` — red test sphere on its own
  (`red_sphere_detector.py`)

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
