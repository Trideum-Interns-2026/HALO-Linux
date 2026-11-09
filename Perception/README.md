# perception

Finds the most confident person in each camera frame using the shared Triton
YOLO server, and publishes it as a `perception_msgs/Detection`.

## Interface

| Direction | Topic | Type | Notes |
| --- | --- | --- | --- |
| in | `/camera/front/image_raw` | `sensor_msgs/Image` | sensor-data QoS (best effort) |
| out | `/perception/detection` | `perception_msgs/Detection` | only published when a person is seen |

## Parameters

| Name | Default | Meaning |
| --- | --- | --- |
| `triton_url` | `$HALO_TRITON_URL`, else `triton:8001` | Triton gRPC address |
| `model_name` | `yolo_person` | Triton model to call |
| `min_confidence` | `0.5` | below this, a frame counts as "no person" |
| `triton_timeout_s` | `0.1` | per-frame inference timeout; slow frames are skipped |

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
