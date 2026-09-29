#!/bin/bash
# One-shot demo: spawns the standing person, bridges the camera from Gazebo,
# launches the YOLO person detector, and opens a live viewer on its debug
# image — everything needed to see detection working, in a single command.
#
# Run this in a NEW terminal, after `fly` is already running SITL + Gazebo
# in another one.
#
# Usage: run_perception_demo.sh [x] [y] [yaw]
#          (person position, default 5 0 0 — see spawn_person.sh; detected
#           with YOLO on the GPU)
#
# Older detectors, not used for the demo — set PERCEIVE_TARGET to one of:
#        person         same person, OpenCV HOG person_detector
#        marked_person  person with a red chest marker, red_sphere_detector
#        sphere         red test sphere, red_sphere_detector
#                       (args: [x] [y] [z], default 3 0 0.3)
set -e
# Job control, so the background `ros2 launch`es below can still be stopped
# with SIGINT. Without it bash starts them with SIGINT ignored, and SIGTERM
# kills launch without stopping its nodes, so nothing shuts them down cleanly.
set -m

source /opt/ros/jazzy/setup.bash
[ -f /opt/ros2_ws/install/setup.bash ] && source /opt/ros2_ws/install/setup.bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

case "${PERCEIVE_TARGET:-yolo_person}" in
    person)
        echo ">> Spawning standing_person..."
        "$SCRIPT_DIR/spawn_person.sh" "$@"
        LAUNCH_FILE=person_detection.launch.py
        DEBUG_IMAGE=/person_detector/person/debug_image
        ;;
    yolo_person)
        echo ">> Spawning standing_person..."
        "$SCRIPT_DIR/spawn_person.sh" "$@"
        LAUNCH_FILE=yolo_person_detection.launch.py
        DEBUG_IMAGE=/yolo_person_detector/person/debug_image
        ;;
    marked_person)
        echo ">> Spawning standing_person with red marker..."
        MARKER=1 "$SCRIPT_DIR/spawn_person.sh" "$@"
        LAUNCH_FILE=perception.launch.py
        DEBUG_IMAGE=/red_sphere_detector/red_sphere/debug_image
        ;;
    sphere)
        echo ">> Spawning red_sphere..."
        "$SCRIPT_DIR/spawn_sphere.sh" "$@"
        LAUNCH_FILE=perception.launch.py
        DEBUG_IMAGE=/red_sphere_detector/red_sphere/debug_image
        ;;
    *)
        echo "Unknown PERCEIVE_TARGET '$PERCEIVE_TARGET' (expected yolo_person, person, marked_person or sphere)" >&2
        exit 1
        ;;
esac

echo ">> Launching Gazebo -> ROS 2 camera bridge in the background..."
# Without this nothing publishes /camera/front/image_raw, and the detector
# just sits waiting for frames.
ros2 launch perception gz_harmonic_bridge.launch.py &
BRIDGE_PID=$!

echo ">> Launching perception node in the background..."
ros2 launch perception "$LAUNCH_FILE" &
PERCEPTION_PID=$!
# Closing the viewer window below also stops the bridge and the
# perception node, so you don't have to hunt down the background processes.
# SIGINT, not the default SIGTERM: on SIGTERM `ros2 launch` exits without
# stopping its nodes, leaving the detector and bridge running (and a second
# `perceive` would then start another detector alongside them).
trap 'kill -INT "$PERCEPTION_PID" "$BRIDGE_PID" 2>/dev/null; wait' EXIT

# Give the node a moment to come up and start publishing, so the topic is
# already there when the viewer opens.
sleep 2

# debug_image_viewer instead of rqt_image_view: rqt subscribes best-effort,
# which drops most of these ~1.5MB frames (see debug_image_viewer.py).
echo ">> Opening viewer on the debug image (q or Esc to close)..."
ros2 run perception debug_image_viewer "$DEBUG_IMAGE"