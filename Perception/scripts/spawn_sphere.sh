#!/bin/bash
# Spawns (or respawns, at a new position) the red_sphere test target in
# Gazebo. Requires the gazebo_ros_factory plugin, which the camera-bridge
# patch already adds to the world.
#
# Usage: spawn_sphere.sh [x] [y] [z]   (defaults: 3 0 0.3)
set -e

X="${1:-3}"
Y="${2:-0}"
Z="${3:-0.3}"

source /opt/ros/foxy/setup.bash

# Delete any existing instance first so re-running this just repositions
# it, instead of erroring on a duplicate entity name.
ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'red_sphere'}" \
    > /dev/null 2>&1 || true

ros2 run gazebo_ros spawn_entity.py \
    -entity red_sphere \
    -file /workspace/Perception/models/red_sphere.sdf \
    -x "$X" -y "$Y" -z "$Z"