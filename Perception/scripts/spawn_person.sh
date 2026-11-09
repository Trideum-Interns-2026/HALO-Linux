#!/bin/bash
# Spawns (or respawns, at a new position) the standing_person test target in
# Gazebo — a static human mesh for the YOLO person detector, via gz-sim's
# entity-factory service.
#
# Also removes a leftover red_sphere if one is in the world.
#
# Usage: spawn_person.sh [x] [y] [yaw]   (defaults: 5 0 0 — facing
#        the drone at origin, far enough out that the whole person is in the
#        camera's view while the drone is still on the ground)
set -e

X="${1:-5}"
Y="${2:-0}"
YAW="${3:-0}"

source /opt/ros/jazzy/setup.bash
source "$(dirname "${BASH_SOURCE[0]}")/gz_world.sh"

for NAME in red_sphere standing_person; do
    gz service -s /world/$WORLD/remove \
        --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 2000 \
        --req "name: \"$NAME\", type: MODEL" \
        > /dev/null 2>&1 || true
done

MODEL_FILE=/workspace/Perception/models/standing_person.sdf

# Half-angle quaternion for a pure yaw rotation.
QZ=$(python3 -c "import math; print(math.sin($YAW / 2))")
QW=$(python3 -c "import math; print(math.cos($YAW / 2))")

# Long timeout: the first spawn downloads the mesh
# from Gazebo Fuel.
gz service -s /world/$WORLD/create \
    --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 30000 \
    --req "sdf_filename: \"$MODEL_FILE\", name: \"standing_person\", pose: {position: {x: $X, y: $Y, z: 0}, orientation: {z: $QZ, w: $QW}}"
