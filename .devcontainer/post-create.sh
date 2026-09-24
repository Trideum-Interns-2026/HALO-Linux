#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash

if [ -d /workspace/Perception ]; then
    mkdir -p /opt/ros2_ws/src
    ln -sfn /workspace/Perception /opt/ros2_ws/src/Perception
    (cd /opt/ros2_ws && colcon build --packages-select perception --symlink-install)
fi

grep -q "alias fly=" /etc/bash.bashrc || \
    echo "alias fly='/usr/local/bin/ros2-entrypoint.sh px4-gazebo'" >> /etc/bash.bashrc
grep -q "source /opt/ros/jazzy/setup.bash" /etc/bash.bashrc || \
    echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc
grep -q "ros2_ws/install/setup.bash" /etc/bash.bashrc || \
    echo "[ -f /opt/ros2_ws/install/setup.bash ] && source /opt/ros2_ws/install/setup.bash" >> /etc/bash.bashrc
