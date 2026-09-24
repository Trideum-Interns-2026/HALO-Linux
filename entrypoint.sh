#!/bin/bash
set -e

TARGET="${1:-${PX4_TARGET:-gazebo-classic_typhoon_h480}}"

# /PX4-Autopilot is a persistent named volume — starts empty on first run.
if [ ! -f "/PX4-Autopilot/Makefile" ]; then
    echo ">> Persistent volume is empty — cloning PX4-Autopilot into it (one-time)..."
    git clone --recurse-submodules https://github.com/PX4/PX4-Autopilot.git /PX4-Autopilot
fi

# Adds the ROS 2 camera bridge plugin to the typhoon_h480 model/world
# (idempotent — no-ops if already patched). Must run before the SDF is
# generated from the .sdf.jinja template, i.e. before `make px4_sitl`.
if [ -f /workspace/Perception/scripts/patch_gazebo_camera_bridge.py ]; then
    python3 /workspace/Perception/scripts/patch_gazebo_camera_bridge.py /PX4-Autopilot
fi

# Build the Perception ROS 2 package so `ros2 run perception
# red_sphere_detector` / the perception launch file are available.
if [ -d /workspace/Perception ]; then
    source /opt/ros/foxy/setup.bash
    # Built outside /workspace so colcon's build/install/log dirs don't
    # land in the repo bind-mount on the host.
    mkdir -p /opt/ros2_ws/src
    ln -sfn /workspace/Perception /opt/ros2_ws/src/Perception
    (cd /opt/ros2_ws && colcon build --packages-select perception --symlink-install)
    source /opt/ros2_ws/install/setup.bash
fi

cd /PX4-Autopilot

# The px4io/px4-dev-ros2-foxy base image already has PX4's build
# dependencies preinstalled, so no ubuntu.sh/pip dance needed here — just
# the mavsdk Python package on top, since that's not part of the base image.
pip3 install --no-cache-dir mavsdk --quiet

if [ "$TARGET" = "bash" ] || [ "$TARGET" = "shell" ]; then
    ln -sfn /workspace/scripts/track /usr/local/bin/track
    grep -q "alias track=" /etc/bash.bashrc || \
        echo "alias track='/workspace/scripts/track'" >> /etc/bash.bashrc
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