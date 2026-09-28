#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash

if [ -d /workspace/Perception ]; then
    mkdir -p /opt/ros2_ws/src
    ln -sfn /workspace/Perception /opt/ros2_ws/src/Perception
    (cd /opt/ros2_ws && colcon build --packages-select perception --symlink-install)
fi

ln -sfn /workspace/scripts/track /usr/local/bin/track

# Make `fly`, ROS 2, and the Perception overlay available in every new
# terminal VS Code opens.
grep -q "alias fly=" /etc/bash.bashrc || \
    echo "alias fly='/usr/local/bin/ros2-entrypoint.sh px4-gazebo'" >> /etc/bash.bashrc
grep -q "source /opt/ros/jazzy/setup.bash" /etc/bash.bashrc || \
    echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc
grep -q "ros2_ws/install/setup.bash" /etc/bash.bashrc || \
    echo "[ -f /opt/ros2_ws/install/setup.bash ] && source /opt/ros2_ws/install/setup.bash" >> /etc/bash.bashrc
grep -q "alias spawn_sphere=" /etc/bash.bashrc || \
    echo "alias spawn_sphere='/workspace/Perception/scripts/spawn_sphere.sh'" >> /etc/bash.bashrc
grep -q "alias perceive=" /etc/bash.bashrc || \
    echo "alias perceive='/workspace/Perception/scripts/run_perception_demo.sh'" >> /etc/bash.bashrc
grep -q "alias track=" /etc/bash.bashrc || \
    echo "alias track='/workspace/scripts/track'" >> /etc/bash.bashrc
