# Drone pipeline (ROS 2, C++17)

Fake inputs -> perception -> geolocation -> POI tracking -> movement (MAVLink seam) -> web endpoint.

## Dataflow / topics

| Topic | Type | From -> To |
|---|---|---|
| /camera/image_raw | sensor_msgs/Image | camera_sim -> perception |
| /sim/target_truth | NavSatFix | world_sim -> perception (sim only) |
| /drone/gps, /drone/battery | NavSatFix, BatteryState | px4_sim -> perception (sim), geolocation, movement, web |
| /perception/detection | drone_msgs/Detection | perception -> geolocation, web |
| /geolocation/estimate | drone_msgs/GeoEstimate | geolocation -> poi_tracker, web |
| /tracking/poi | drone_msgs/Poi | poi_tracker -> movement, web |
| /sim/mavlink_out | drone_msgs/MoveCommand | movement -> px4_sim, web |

Sim-only nodes: world_sim, camera_sim, px4_sim. Replace them with the real camera and PX4.
The real swap points are `Detector` (perception_node.cpp) and `MavlinkLink` (movement_node.cpp).

## Build and run (ROS 2 Humble or Jazzy)

    cd drone_ws
    colcon build
    source install/setup.bash
    ros2 launch drone_pipeline pipeline.launch.py

Then open http://localhost:8080 (or http://<laptop-ip>:8080 from a phone) and try
`ros2 topic echo /tracking/poi`.

## Known simplifications
- Camera is nadir (straight down), image-up = north, no drone attitude in geolocation.
- Geolocation uses the latest GPS fix instead of interpolating to the detection timestamp.
- ECEF error is an axis-aligned 1-sigma per axis, not a full covariance.
- The MAVLink link is a ROS topic stub; altitude frames (ellipsoid vs AMSL) must be reconciled for real PX4.
