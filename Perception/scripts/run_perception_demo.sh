#!/bin/bash
# One-shot demo: spawns the test sphere, bridges the camera from Gazebo,
# launches the perception node, and
# opens rqt_image_view on its debug image — everything needed to see
# detection working, in a single command.
#
# Run this in a NEW terminal, after `fly` is already running SITL + Gazebo
# in another one.
#
# Usage: run_perception_demo.sh [x] [y] [z]   (sphere position, default 3 0 0.3)
#        PERCEIVE_TARGET=person run_perception_demo.sh [x] [y] [yaw]
#          (spawns the standing person and runs person_detector instead —
#           see spawn_person.sh for its defaults)
#        PERCEIVE_TARGET=yolo_person run_perception_demo.sh [x] [y] [yaw]
#          (same person, detected with YOLO on the GPU instead of HOG)
#        PERCEIVE_TARGET=marked_person run_perception_demo.sh [x] [y] [yaw]
#          (standing person with a red chest marker, red_sphere_detector)
set -e

source /opt/ros/jazzy/setup.bash
[ -f /opt/ros2_ws/install/setup.bash ] && source /opt/ros2_ws/install/setup.bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

case "${PERCEIVE_TARGET:-sphere}" in
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
    *)
        echo ">> Spawning red_sphere..."
        "$SCRIPT_DIR/spawn_sphere.sh" "$@"
        LAUNCH_FILE=perception.launch.py
        DEBUG_IMAGE=/red_sphere_detector/red_sphere/debug_image
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
# Closing the rqt_image_view window below also stops the bridge and the
# perception node, so you don't have to hunt down the background processes.
trap 'kill "$PERCEPTION_PID" "$BRIDGE_PID" 2>/dev/null' EXIT

# Give the node a moment to come up and start publishing, so the topic is
# already there when the viewer opens.
sleep 2

echo ">> Opening rqt_image_view on the debug image..."
ros2 run rqt_image_view rqt_image_view "$DEBUG_IMAGE"