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

- The camera bridge (`Perception/scripts/patch_gazebo_camera_bridge.py`) adds
  a `gazebo_ros` camera plugin to the typhoon_h480 model so Gazebo publishes
  the camera feed as a ROS 2 topic — `/camera/front/image_raw` (the `front`
  segment comes from the plugin's `camera_name`, not `/camera/image_raw`).
  This patch runs automatically: `entrypoint.sh` runs it for the standalone
  `run.py` path, and `.devcontainer/post-create.sh` runs it for the VS Code
  dev container path — they're separate flows that don't share setup, so
  anything added to one needs adding to the other too.
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

1. `fly` — launches Gazebo.
2. `perceive` — runs perception.
3. `track` — runs the Kalman filter and repeatedly sends the filtered target
  location to QGroundControl as `MAV_CMD_DO_SET_ROI_LOCATION`, keeping the
  QGroundControl ROI command updated while the target is visible. The same
  terminal displays the drone and filtered target coordinates once per second.

Run `track` in a separate terminal after `perceive` is running. The vehicle
must be connected on MAVSDK UDP `14540`. QGroundControl does not persistently
draw arbitrary map pins for ROI commands, so use the `track` terminal readout
for the live target latitude, longitude, altitude, and distance.
