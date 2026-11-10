# tracking

Estimates the detected person's world position from `perception_msgs/Detection`, camera
intrinsics, Geolocation's drone position, and PX4 attitude. Range is estimated internally from
the person bounding-box height; it is not published. The camera bounding-box ray is transformed
from optical coordinates to PX4 body FRD, then by the PX4 FRD-to-NED attitude and into WGS84 LLA.
Height and pixel uncertainty plus Geolocation uncertainty produce per-axis ECEF 3-sigma bounds.

The default camera mount is aligned with the vehicle's forward FRD axis. Configure its fixed
mounting roll, pitch and yaw if the physical/simulated mount differs. PX4 `VehicleAttitude.q` is
Hamilton `[w,x,y,z]`, body FRD to earth NED.

## Topics

| Direction | Topic | Type | QoS |
| --- | --- | --- | --- |
| in | `/perception/detection` | `perception_msgs/Detection` | sensor-data |
| in | `/camera/front/camera_info` | `sensor_msgs/CameraInfo` | sensor-data |
| in | `/geolocation/position` | `geolocation_msgs/GeolocationState` | reliable |
| in | `/fmu/out/vehicle_attitude` | `px4_msgs/VehicleAttitude` | sensor-data |
| out | `/tracking/position` | `tracking_msgs/TrackingState` | reliable, depth 10 |

Outputs copy the Detection header and are published only when a valid detection and fresh
position/attitude inputs are available. The output error fields are ECEF X/Y/Z 3-sigma bounds in
metres.

## Parameters

| Name | Default | Meaning |
| --- | --- | --- |
| `detection_topic` | `/perception/detection` | Person Detection input |
| `camera_info_topic` | `/camera/front/camera_info` | Camera calibration input |
| `geolocation_topic` | `/geolocation/position` | Drone position input |
| `vehicle_attitude_topic` | `/fmu/out/vehicle_attitude` | PX4 FRD-to-NED attitude input |
| `output_topic` | `/tracking/position` | Person position output |
| `human_height_m` | `1.75` | Assumed person height for pinhole ranging |
| `human_height_stddev_m` | `0.15` | Height uncertainty propagated to output bounds |
| `pixel_stddev_px` | `2.0` | Bounding-box centre uncertainty |
| `minimum_confidence` | `0.5` | Detections below this value are rejected |
| `maximum_input_age_s` | `0.5` | Maximum age of position, attitude and detection |
| `camera_roll_deg` | `0.0` | Camera mounting roll relative to vehicle FRD |
| `camera_pitch_deg` | `0.0` | Camera mounting pitch relative to vehicle FRD |
| `camera_yaw_deg` | `0.0` | Camera mounting yaw relative to vehicle FRD |
| `frame_edge_margin_px` | `12.0` | Boxes with `y_min <= margin` or `y_max >= height - margin` are not published; clamped to 0..100 |

Detections whose box touches the top or bottom image edge (within `frame_edge_margin_px`) are
dropped: the clipped box is too short, so pinhole ranging would place the person too far away.
The image height comes from `camera_info.height`; if it is 0 (unknown), nothing counts as clipped.
The default margin is 12 px because a truncated person's box edge lands a few model pixels inside the
frame and the 1920x1080 image is letterboxed to 640x640 (1 model pixel = 3 image pixels).

## Run

```bash
source /opt/ros/jazzy/setup.bash
source /opt/px4_ros2/install/setup.bash
cd /opt/ros2_ws && colcon build \
  --packages-select perception_msgs geolocation_msgs tracking_msgs tracking --symlink-install
source /opt/ros2_ws/install/setup.bash
ros2 launch tracking tracking.launch.py
```

The PX4 ROS 2 bridge must publish `/fmu/out/vehicle_attitude`; no MAVLink connection is opened
by Tracking.

Override parameters with
`ros2 run tracking tracking_node --ros-args -p minimum_confidence:=0.7`.

## Test

```bash
cd /opt/ros2_ws && colcon test --packages-select tracking_msgs tracking
colcon test-result --verbose
```
