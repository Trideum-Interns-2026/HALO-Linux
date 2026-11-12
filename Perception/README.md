# perception

Finds the most confident person in each camera frame using the shared Triton
YOLO server, and publishes it as a `perception_msgs/Detection`.

## Interface

| Direction | Topic | Type | Notes |
| --- | --- | --- | --- |
| in | `/camera/front/image_raw` | `sensor_msgs/Image` | reliable, depth 1 (exception to sensor-data QoS, see below) |
| out | `/perception/detection` | `perception_msgs/Detection` | only published when a person is seen |

## Parameters

| Name | Default | Meaning |
| --- | --- | --- |
| `triton_url` | `$HALO_TRITON_URL`, else `triton:8001` | Triton gRPC address |
| `model_name` | `yolo_person` | Triton model to call |
| `min_confidence` | `0.5` | below this, a frame counts as "no person" |
| `triton_timeout_s` | `0.1` | per-frame inference timeout; slow frames are skipped |
| `image_topic` | `/camera/front/image_raw` | camera image input (QoS: see `image_reliability`) |
| `detection_topic` | `/perception/detection` | where `Detection` is published |
| `image_reliability` | `reliable` | `reliable` or `best_effort`; must match the camera publisher |

Why the image is reliable: a 1920x1080 frame is 6 MB, fragmented over UDP. A best-effort reader
lost whole frames and saw about 4 Hz of the sim camera's 15 Hz (so detections came at 3.5 Hz); a
reliable depth-1 reader gets all 15 Hz. A reliable reader cannot connect to a best-effort
publisher, so set `image_reliability:=best_effort` for a camera driver that publishes that way.

## Detection overlay (debug view)

`detection_overlay` is a second node with one job: draw the detected box on the camera image so you
can watch what Perception sees. It is off by default; start it with `overlay:=true`.

| Direction | Topic | Type | Notes |
| --- | --- | --- | --- |
| in | `/camera/front/image_raw` | `sensor_msgs/Image` | same QoS rule as the detector (`image_reliability`) |
| in | `/perception/detection` | `perception_msgs/Detection` | newest box, drawn on the newest image |
| out | `/perception/debug_image` | `sensor_msgs/Image` | bgr8, scaled, header copied from the camera image |

| Name | Default | Meaning |
| --- | --- | --- |
| `image_topic` | `/camera/front/image_raw` | camera image input |
| `detection_topic` | `/perception/detection` | detector output to draw |
| `output_topic` | `/perception/debug_image` | the picture with the box |
| `image_reliability` | `reliable` | `reliable` or `best_effort`; must match the camera publisher |
| `scale` | `0.5` | output size factor, 0.05 to 1.0 (a 1080p frame is shrunk so the viewer stays light) |
| `max_age_s` | `0.5` | a box further than this from the image's timestamp is not drawn |

View it: `ros2 launch perception perception.launch.py overlay:=true` and
`ros2 run rqt_image_view rqt_image_view /perception/debug_image`. `sim_check --gui` does both.

## Run

```
fly                                   # terminal 1: sim
perceive                              # terminal 2: bridge + node (ros2 launch perception perception.launch.py)
spawn_person                          # optional: a person 5 m ahead
ros2 topic echo /perception/detection
```

Override a parameter: `ros2 run perception yolo_person_detector --ros-args -p min_confidence:=0.7`

## Test

```
source /opt/ros2_ws/install/setup.bash
cd /workspace/Perception && python3 -m pytest test/ -v
```
CI runs the same tests on every push: `.github/workflows/perception-tests.yml`.
