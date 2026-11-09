# geolocation

Filters the drone's own GPS position. It consumes `sensor_msgs/NavSatFix`, uses the diagonal terms
of the GPS covariance when available (configured standard deviations otherwise), filters in a
local ENU tangent frame, and publishes WGS84 LLA with per-axis 3-sigma bounds rotated into ECEF.
Geolocation does not consume perception data.

## Topics

| Direction | Topic | Type | QoS |
| --- | --- | --- | --- |
| in | `/gps/fix` | `sensor_msgs/NavSatFix` | sensor-data |
| out | `/geolocation/position` | `geolocation_msgs/GeolocationState` | reliable, depth 10 |

The output copies the GPS header. `altitude_m` is WGS84 ellipsoid altitude. The `error_*_m`
fields are 3-sigma bounds along ECEF X/Y/Z, in metres.

## Parameters

| Name | Default | Meaning |
| --- | --- | --- |
| `gps_topic` | `/gps/fix` | GPS fix input topic |
| `output_topic` | `/geolocation/position` | Filtered position output topic |
| `horizontal_stddev_m` | `1.0` | Fallback east/north GPS standard deviation |
| `vertical_stddev_m` | `1.5` | Fallback up GPS standard deviation |
| `process_noise_variance_m2_per_s` | `0.25` | Random-walk process variance per second |

## Run

```bash
source /opt/ros/jazzy/setup.bash
cd /opt/ros2_ws && colcon build --packages-select geolocation_msgs geolocation --symlink-install
source /opt/ros2_ws/install/setup.bash
ros2 launch geolocation geolocation.launch.py
```

The GPS driver/bridge must publish `sensor_msgs/NavSatFix` on `/gps/fix` (or configure
`gps_topic`).

Override a topic with
`ros2 run geolocation geolocation_node --ros-args -p gps_topic:=/custom/gps/fix`.

## Test

```bash
cd /opt/ros2_ws && colcon test --packages-select geolocation_msgs geolocation
colcon test-result --verbose
```
