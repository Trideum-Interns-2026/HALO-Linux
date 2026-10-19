#!/bin/bash
# Spawns (or respawns, at a new position) the red_sphere test target in
# Gazebo, via gz-sim's native UserCommands entity-factory service (loaded
# by server.config) — Harmonic has no gazebo_ros/gazebo_msgs, so the
# classic spawn_entity.py service call this used before doesn't exist here.
#
# Usage: spawn_sphere.sh [x] [y] [z]   (defaults: 3 0 0.3)
set -e

X="${1:-3}"
Y="${2:-0}"
Z="${3:-0.3}"

source /opt/ros/jazzy/setup.bash
source "$(dirname "${BASH_SOURCE[0]}")/gz_world.sh"

# Delete any existing instance first so re-running this just repositions
# it, instead of erroring on a duplicate entity name.
gz service -s /world/$WORLD/remove \
    --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 2000 \
    --req 'name: "red_sphere", type: MODEL' \
    > /dev/null 2>&1 || true

gz service -s /world/$WORLD/create \
    --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 2000 \
    --req "sdf_filename: \"/workspace/Perception/models/red_sphere.sdf\", name: \"red_sphere\", pose: {position: {x: $X, y: $Y, z: $Z}}"