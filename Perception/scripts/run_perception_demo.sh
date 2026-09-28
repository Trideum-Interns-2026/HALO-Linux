#!/bin/bash
# One-shot demo: spawns the test sphere, launches the perception node, and
# opens rqt_image_view on its debug image — everything needed to see
# detection working, in a single command.
#
# Run this in a NEW terminal, after `fly` is already running SITL + Gazebo
# in another one.
#
# Usage: run_perception_demo.sh [x] [y] [z]   (sphere position, default 3 0 0.3)
set -e

source /opt/ros/foxy/setup.bash
[ -f /opt/ros2_ws/install/setup.bash ] && source /opt/ros2_ws/install/setup.bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo ">> Spawning red_sphere..."
"$SCRIPT_DIR/spawn_sphere.sh" "$@"

echo ">> Launching perception node in the background..."
ros2 launch perception perception.launch.py &
PERCEPTION_PID=$!
# Closing the rqt_image_view window below also stops the perception node,
# so you don't have to hunt down the background process yourself.
trap 'kill "$PERCEPTION_PID" 2>/dev/null' EXIT

# Give the node a moment to come up and start publishing, so the topic is
# already there when the viewer opens.
sleep 2

echo ">> Opening rqt_image_view on the debug image..."
ros2 run rqt_image_view rqt_image_view /red_sphere_detector/red_sphere/debug_image