#!/bin/bash
# Runs once when the dev container is created. The container's entrypoint.sh
# (camera-bridge patch, colcon build, ROS sourcing) never runs here — the
# dev container overrides the entrypoint with `sleep infinity` (see
# docker-compose.extend.yml) — so this script does the equivalent setup for
# VS Code's terminals.
set -e

# /PX4-Autopilot is a persistent named volume — starts empty on first run.
if [ ! -f /PX4-Autopilot/Makefile ]; then
    git clone --recurse-submodules https://github.com/PX4/PX4-Autopilot.git /PX4-Autopilot
fi

# Camera bridge for the Perception package (idempotent — safe to re-run).
if [ -f /workspace/Perception/scripts/patch_gazebo_camera_bridge.py ]; then
    python3 /workspace/Perception/scripts/patch_gazebo_camera_bridge.py /PX4-Autopilot
fi

# Build the Perception ROS 2 package into /opt/ros2_ws (kept outside
# /workspace so colcon's build/install/log dirs don't land in the repo).
if [ -d /workspace/Perception ]; then
    source /opt/ros/foxy/setup.bash
    mkdir -p /opt/ros2_ws/src
    ln -sfn /workspace/Perception /opt/ros2_ws/src/Perception
    (cd /opt/ros2_ws && colcon build --packages-select perception --symlink-install)
fi

ln -sfn /workspace/scripts/track /usr/local/bin/track

# Make `fly`, ROS 2, and the Perception overlay available in every new
# terminal VS Code opens (they don't inherit anything from entrypoint.sh).
grep -q "alias fly=" /etc/bash.bashrc || \
    echo "alias fly='cd /PX4-Autopilot && make px4_sitl gazebo-classic_typhoon_h480'" >> /etc/bash.bashrc
grep -q "source /opt/ros/foxy/setup.bash" /etc/bash.bashrc || \
    echo "source /opt/ros/foxy/setup.bash" >> /etc/bash.bashrc
grep -q "ros2_ws/install/setup.bash" /etc/bash.bashrc || \
    echo "[ -f /opt/ros2_ws/install/setup.bash ] && source /opt/ros2_ws/install/setup.bash" >> /etc/bash.bashrc
    grep -q "alias spawn_sphere=" /etc/bash.bashrc || \
    echo "alias spawn_sphere='/workspace/Perception/scripts/spawn_sphere.sh'" >> /etc/bash.bashrc
    grep -q "alias perceive=" /etc/bash.bashrc || \
    echo "alias perceive='/workspace/Perception/scripts/run_perception_demo.sh'" >> /etc/bash.bashrc
    grep -q "alias track=" /etc/bash.bashrc || \
        echo "alias track='/workspace/scripts/track'" >> /etc/bash.bashrc