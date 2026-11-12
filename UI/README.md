# ui

The UI service's ROS side. A bridge node that reads the drone position (Geolocation) and the
person position (Tracking), translates them to JSON and posts them over HTTP to the ground-station
web API in [`GroundstationAPI/`](../GroundstationAPI/README.md), which serves the website.
The UI never talks to ROS or the vehicle directly, and this node never commands the vehicle.

Not yet published: heading, ground speed, battery, flight mode and distance to the person. No
service publishes them (range is internal to Tracking), so the dashboard shows "—" for them.

## Topics

| Direction | Topic | Type | QoS |
| --- | --- | --- | --- |
| in | `/geolocation/position` | `geolocation_msgs/GeolocationState` | reliable, depth 10 |
| in | `/tracking/position` | `tracking_msgs/TrackingState` | reliable, depth 10 |
| out | HTTP `POST` to `api_url` | JSON, see the web API's `Telemetry` model | n/a |

Nothing is posted until there is a fresh drone position. The person's fields are left out when
Tracking has no fresh position. Error bounds are not forwarded yet.

## Parameters

| Name | Default | Meaning |
| --- | --- | --- |
| `geolocation_topic` | `/geolocation/position` | Drone position input |
| `tracking_topic` | `/tracking/position` | Person position input |
| `api_url` | `http://127.0.0.1:8000/api/telemetry` | Web API endpoint (launch arg `api_url:=`) |
| `post_rate_hz` | `2.0` | How often the latest positions are posted |
| `maximum_input_age_s` | `2.0` | Older inputs are dropped (compared with the node clock) |
| `http_timeout_s` | `1.0` | Per-request timeout; the POST runs off the ROS thread |

If the web API is down the node logs a throttled warning and keeps running.

## Run and test

```bash
groundstation                                   # terminal 1: web API and dashboard on 127.0.0.1:8000
ui                                              # terminal 2 (alias for ros2 launch ui ui.launch.py)
# in the sim add use_sim_time:=true, as for tracking

cd /opt/ros2_ws && colcon build --packages-select geolocation_msgs tracking_msgs ui --symlink-install
python3 -m pytest UI/test/ -v                   # from the repo root, workspace sourced
```
