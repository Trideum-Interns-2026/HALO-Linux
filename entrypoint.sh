#!/bin/bash
set -e

TARGET="${1:-${PX4_TARGET:-gazebo-classic_typhoon_h480}}"

# /PX4-Autopilot is a persistent named volume — starts empty on first run.
if [ ! -f "/PX4-Autopilot/Makefile" ]; then
    echo ">> Persistent volume is empty — cloning PX4-Autopilot into it (one-time)..."
    git clone --recurse-submodules https://github.com/PX4/PX4-Autopilot.git /PX4-Autopilot
fi

cd /PX4-Autopilot

# The px4io/px4-dev-ros2-foxy base image already has PX4's build
# dependencies preinstalled, so no ubuntu.sh/pip dance needed here — just
# the mavsdk Python package on top, since that's not part of the base image.
pip3 install --no-cache-dir mavsdk --quiet

if [ "$TARGET" = "bash" ] || [ "$TARGET" = "shell" ]; then
    exec /bin/bash
fi

echo ">> Launching PX4 SITL with target: $TARGET"
echo ">> QGroundControl should auto-connect on UDP 14550"
echo ">> Gazebo should appear on your display"
echo

if [ ! -f "/PX4-Autopilot/build/px4_sitl_default/bin/px4" ]; then
    echo ">> First build — configuring px4_sitl_default first..."
    make px4_sitl_default
fi

exec make px4_sitl "$TARGET"