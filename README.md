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
  internet once). `standing_person_marker.sdf` is the same person with a
  0.4m red sphere on the chest, for the red sphere detector. Spawn with
  `spawn_person [x] [y] [yaw]` (default 5 0 0, facing the drone), or
  `MARKER=1 spawn_person` for the marked one. Spawning it removes
  `red_sphere`, so the detectors only ever see one target.

To select a target type, set `PERCEIVE_TARGET` for both `perceive` and
`track`:

- `PERCEIVE_TARGET=person` — unmarked person, `person_detector`
- `PERCEIVE_TARGET=marked_person` — person with red marker,
  `red_sphere_detector`

## Running it

1. Open QGroundControl.
2. In a terminal in the Dev Container, run `fly`. It starts PX4 SITL and
   Gazebo only; leave it running.
3. In another terminal, run `perceive`. It spawns the selected test target,
   starts the Gazebo camera bridge and detector, and opens the debug-image
   viewer. Leave it running while tracking.
4. In a third terminal, run `track`. It builds and runs `main`, which starts
   the geographic Kalman tracker from
   `Include/Tracking/target_kalman_filter.hpp` and the camera-based centering
   node from `Include/Tracking/track_and_center.hpp` together. The Kalman
   tracker publishes MAVLink position/ROI updates; the centering node uses
   the camera bearing to yaw the drone toward the detected target. Arm and
   take off the vehicle yourself before centering; `track` does not arm, take
   off, or land. If `track` is already running, centering starts when PX4
   reports the vehicle armed. If it is disarmed, centering stops until it is
   armed again. Press Ctrl+C in the `track` terminal to stop tracking;
   `fly` and `perceive` remain running until stopped in their own terminals.

The default target is a red sphere. To track an unmarked person or a marked
person, set the same target selection in the `perceive` and `track` terminals.
For example, run `PERCEIVE_TARGET=person perceive` and
`PERCEIVE_TARGET=person track`. Use `PERCEIVE_TARGET=marked_person` for the
person with the red chest marker.

Tracking is intended for PX4/Gazebo SITL only; do not run it against a real
vehicle. Arm and take off using QGroundControl or the PX4 SITL controls first.
QGroundControl and the vehicle must be connected through MAVLink, and the
tracking nodes use MAVSDK UDP `14540`.

### Docker resources

The Compose files do not set CPU or storage limits. `fly` runs inside the
already-created Dev Container; it cannot allocate more CPU or disk to Docker.
If Docker reports zero CPUs or the container exits from resource pressure,
check Docker Desktop's WSL2 resource allocation and available host/WSL disk
space before rerunning. Read-only checks include `docker info`,
`docker system df`, `free -h`, and `df -h`; avoid pruning Docker data unless
you have confirmed it is safe to remove.
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

The Spark is a shared, headless host: each teammate connects from their
laptop through plain SSH + VS Code Remote-SSH, opens their own clone, and
"Reopen in Container" with the **PX4 SITL Dev Container (NVIDIA Spark)**
profile. Gazebo and QGroundControl stream to your browser (Selkies,
GPU-encoded — QGC is started for you inside the container, since the sim's
MAVLink never leaves it) and YOLO runs on one shared TensorRT engine behind
NVIDIA Triton, so five people's sims don't each load their own model.

Setup and day-to-day usage: [docs/spark/README.md](docs/spark/README.md).
