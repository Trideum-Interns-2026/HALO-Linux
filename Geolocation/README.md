# geolocation

Filters the drone's own GPS position. It consumes `sensor_msgs/NavSatFix`, uses the diagonal terms
of the GPS covariance when available (configured standard deviations otherwise), filters in a
local ENU tangent frame, and publishes WGS84 LLA with per-axis 3-sigma bounds rotated into ECEF.
The `building_risk` node independently uses that position and building data to estimate overlap
with nearby building footprints. Geolocation does not consume perception data.

## Topics

| Direction | Topic | Type | QoS |
| --- | --- | --- | --- |
| in | `/gps/fix` | `sensor_msgs/NavSatFix` | sensor-data |
| out | `/geolocation/position` | `geolocation_msgs/GeolocationState` | reliable, depth 10 |
| in | `/geolocation/position` | `geolocation_msgs/GeolocationState` | reliable, depth 10 |
| out | `/geolocation/building_risk` | `geolocation_msgs/BuildingRisk` | reliable, depth 10 |

The output copies the GPS header. `altitude_m` is WGS84 ellipsoid altitude. The `error_*_m`
fields are 3-sigma bounds along ECEF X/Y/Z, in metres.
`building_risk` copies that header and publishes only when a building is in range and the
position's lowest 3-sigma altitude does not clear its roof. The risk estimate uses a local ENU
rotation of the ECEF error bounds to calculate horizontal uncertainty. The current CSV footprints
are synthetic placeholders, not authoritative Huntsville building data.

## Parameters

| Name | Default | Meaning |
| --- | --- | --- |
| `gps_topic` | `/gps/fix` | GPS fix input topic |
| `output_topic` | `/geolocation/position` | Filtered position output topic |
| `horizontal_stddev_m` | `1.0` | Fallback east/north GPS standard deviation |
| `vertical_stddev_m` | `1.5` | Fallback up GPS standard deviation |
| `process_noise_variance_m2_per_s` | `0.25` | Random-walk process variance per second |
| `input_topic` | `/geolocation/position` | `building_risk` position input |
| `output_topic` | `/geolocation/building_risk` | `building_risk` estimate output |
| `buildings_file` | installed `config/buildings_huntsville.csv` | Building disc data in CSV format |
| `max_range_m` | `200.0` | Maximum horizontal distance from position to building-disc edge |

Building CSV columns are `id,latitude_deg,longitude_deg,radius_m,base_alt_m,height_m`. Each
footprint is currently a horizontal disc; roof altitude is `base_alt_m + height_m`, in metres
above the WGS84 ellipsoid. Invalid rows are skipped with a warning. A missing file or invalid
header produces no building-risk output and a warning.

## Run

```bash
source /opt/ros/jazzy/setup.bash
cd /opt/ros2_ws && colcon build --packages-select geolocation_msgs geolocation --symlink-install
source /opt/ros2_ws/install/setup.bash
ros2 launch geolocation geolocation.launch.py
```

The GPS driver/bridge must publish `sensor_msgs/NavSatFix` on `/gps/fix` (or configure
`gps_topic`). The launch also starts `building_risk_node`; override its parameters directly with
`ros2 run geolocation building_risk_node --ros-args -p buildings_file:=/path/to/buildings.csv`.

Override a topic with
`ros2 run geolocation geolocation_node --ros-args -p gps_topic:=/custom/gps/fix`.

## Test

```bash
cd /opt/ros2_ws && colcon test --packages-select geolocation_msgs geolocation
colcon test-result --verbose
```

## Tuning

`config/tuned.yaml` has a process noise tuned with `geolocation_tune` (built with the package, no
ROS or sim needed): `ros2 run geolocation geolocation_tune`. The simulator's GPS is noiseless, so
the tuner uses the noise of a real receiver (1.0 m horizontal, 1.5 m vertical) on a hovering and an
orbiting drone, and scores error against the known truth. The filter is a random walk, so it lags a
moving drone; only a velocity state would remove that. Retune when the receiver, its rate or the
vehicle's motion changes.
