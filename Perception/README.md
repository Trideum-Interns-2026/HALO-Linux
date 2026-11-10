# perception

Finds the person in each camera frame using the shared Triton YOLO server, and publishes it as a
`perception_msgs/Detection`. Built for 30 Hz: a running person moves about 15 cm between frames at
30 Hz (30 cm at 15 Hz), and Tracking and Movement cannot use more than that.

## Interface

| Direction | Topic | Type | Notes |
| --- | --- | --- | --- |
| in | `/camera/front/image_raw` | `sensor_msgs/Image` | reliable, depth 1 (exception to sensor-data QoS, see below) |
| out | `/perception/detection` | `perception_msgs/Detection` | only published for a confirmed person; the box is smoothed |

## Parameters

| Name | Default | Meaning |
| --- | --- | --- |
| `triton_url` | `$HALO_TRITON_URL`, else `triton:8001` | Triton gRPC address |
| `model_name` | `yolo_person` | Triton model to call |
| `min_confidence` | `0.7` | score needed to START tracking a person (trees scored about 0.6) |
| `keep_confidence` | `0.5` | score that keeps an already confirmed person tracked (never above `min_confidence`) |
| `confirm_frames` | `3` | consecutive overlapping frames at `min_confidence` before the first `Detection` (about 66 ms at 30 Hz) |
| `max_missed_frames` | `5` | frames without a match before the track is dropped; nothing is published while missed |
| `match_iou` | `0.2` | box overlap with the previous frame that counts as the same person |
| `min_box_height_frac` | `0.03` | boxes shorter than this fraction of the frame height are not people |
| `min_aspect`, `max_aspect` | `1.2`, `6.0` | allowed box height / width; wide, round shapes (tree canopies) are rejected |
| `smooth_box` | `true` | alpha-beta filter on the box (jitter down, no lag on a moving person) |
| `smoothing_alpha`, `smoothing_beta` | `0.7`, `0.3` | position and velocity gain; alpha 1 = no smoothing |
| `async_inference` | `true` | process frames on a worker thread that keeps only the newest frame (old ones are dropped, never queued) |
| `timing_report_s` | `0.0` | `> 0`: log fps and mean/max ms of preprocess, triton, decode and total every this many seconds |
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
| out | `/perception/debug_image/compressed` | `sensor_msgs/CompressedImage` | JPEG, only with `publish_compressed` (on from the launch file) |

| Name | Default | Meaning |
| --- | --- | --- |
| `image_topic` | `/camera/front/image_raw` | camera image input |
| `detection_topic` | `/perception/detection` | detector output to draw |
| `output_topic` | `/perception/debug_image` | the picture with the box |
| `image_reliability` | `reliable` | `reliable` or `best_effort`; must match the camera publisher |
| `scale` | `0.5` | output size factor, 0.05 to 1.0 (a 1080p frame is shrunk so the viewer stays light) |
| `max_age_s` | `0.5` | a box further than this from the image's timestamp is not drawn |
| `max_predict_s` | `0.15` | the box is moved to each image's timestamp at constant velocity, at most this far ahead of the newest detection |
| `publish_compressed` | `false` (`compressed:=true` in the launch file) | also publish JPEG on `<output_topic>/compressed` |
| `jpeg_quality` | `80` | JPEG quality 0-100 |

View it: start the stack with `overlay:=true` and run
`ros2 run rqt_image_view rqt_image_view /perception/debug_image/compressed` (the JPEG topic is much
smoother than the raw one). `sim_check --gui` does both.

## Run

```
fly                                   # terminal 1: sim
perceive                              # terminal 2: bridge + node (ros2 launch perception perception.launch.py)
spawn_person                          # optional: a person 5 m ahead
ros2 topic echo /perception/detection
```

Override a parameter: `ros2 run perception yolo_person_detector --ros-args -p min_confidence:=0.8`

## Measure and tune (30 Hz budget = 33 ms per frame)

```
python3 Perception/scripts/bench_cpu_stages.py     # CPU stages on 1080p frames, no sim/Triton (about 2.5 ms)
perceive timing_report_s:=5.0                      # live: fps and ms per stage in the detector log
```
The detector logs `timing: 30.0 fps (mean/max) | decode ..., preprocess ..., total ..., triton ...`.
If `triton` is above about 15 ms, serve a smaller model (`MODEL=yolov8m ./infra/triton/build_engine.sh`).
Tree false positives: raise `min_confidence` toward the gap between tree and person scores, or
tighten `min_aspect`. A box that lags on a fast person: raise `smoothing_alpha`/`smoothing_beta`.
Frames dropped by the worker show up as a detector fps below the camera fps.

## Test

```
source /opt/ros2_ws/install/setup.bash
cd /workspace/Perception && python3 -m pytest test/ -v
```
CI runs the same tests on every push: `.github/workflows/perception-tests.yml`.
