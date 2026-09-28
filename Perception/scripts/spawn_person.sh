#!/bin/bash
# Spawns (or respawns, at a new position) the standing_person test target in
# Gazebo — a static human mesh for person_detector. With MARKER=1 it gets a
# red chest marker instead, which red_sphere_detector tracks as-is. Same
# gz-sim entity-factory approach as spawn_sphere.sh.
#
# Also removes red_sphere if it's in the world, since the detectors would
# otherwise have two candidate targets to jump between.
#
# Usage: [MARKER=1] spawn_person.sh [x] [y] [yaw]   (defaults: 5 0 0 — facing
#        the drone at origin, far enough out that the whole person is in the
#        camera's view while the drone is still on the ground)
set -e

X="${1:-5}"
Y="${2:-0}"
YAW="${3:-0}"

source /opt/ros/jazzy/setup.bash

for NAME in red_sphere standing_person; do
    gz service -s /world/default/remove \
        --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 2000 \
        --req "name: \"$NAME\", type: MODEL" \
        > /dev/null 2>&1 || true
done

if [ "${MARKER:-0}" = "1" ]; then
    MODEL_FILE=/workspace/Perception/models/standing_person_marker.sdf
else
    MODEL_FILE=/workspace/Perception/models/standing_person.sdf
fi

# Half-angle quaternion for a pure yaw rotation.
QZ=$(python3 -c "import math; print(math.sin($YAW / 2))")
QW=$(python3 -c "import math; print(math.cos($YAW / 2))")

# Longer timeout than spawn_sphere.sh: the first spawn downloads the mesh
# from Gazebo Fuel.
gz service -s /world/default/create \
    --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 30000 \
    --req "sdf_filename: \"$MODEL_FILE\", name: \"standing_person\", pose: {position: {x: $X, y: $Y, z: 0}, orientation: {z: $QZ, w: $QW}}"
