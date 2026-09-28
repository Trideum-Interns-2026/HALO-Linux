#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash

if [ -d /workspace/Perception ]; then
    mkdir -p /opt/ros2_ws/src
    ln -sfn /workspace/Perception /opt/ros2_ws/src/Perception
    (cd /opt/ros2_ws && colcon build --packages-select perception --symlink-install)
fi

ln -sfn /workspace/scripts/track /usr/local/bin/track

# The bind-mounted repo is owned by the host user (uid 1000) but the container
# runs as root, so git refuses it as "dubious ownership" without this.
git config --global --get-all safe.directory | grep -qx /workspace || \
    git config --global --add safe.directory /workspace

# Make `fly`, ROS 2, and the Perception overlay available in every new
# terminal VS Code opens.
#
# Defaults to gz_x500_depth (X500 + OakD-Lite depth camera) rather than
# PX4's plain gz_x500, since Perception needs the camera feed to work at
# all. Still overridable by exporting PX4_SIM_MODEL before running `fly`.
#
# The base image bakes in PX4_SIM_MODEL=gz_x500, so the `:-` fallback in the
# alias alone never kicks in — export the depth model explicitly first (the
# Dockerfile does too, this covers containers built before that change).
grep -q "export PX4_SIM_MODEL=" /etc/bash.bashrc || \
    echo "export PX4_SIM_MODEL=gz_x500_depth" >> /etc/bash.bashrc
grep -q "alias fly=" /etc/bash.bashrc || \
    echo "alias fly='PX4_SIM_MODEL=\${PX4_SIM_MODEL:-gz_x500_depth} /usr/local/bin/ros2-entrypoint.sh px4-gazebo'" >> /etc/bash.bashrc
grep -q "source /opt/ros/jazzy/setup.bash" /etc/bash.bashrc || \
    echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc
grep -q "ros2_ws/install/setup.bash" /etc/bash.bashrc || \
    echo "[ -f /opt/ros2_ws/install/setup.bash ] && source /opt/ros2_ws/install/setup.bash" >> /etc/bash.bashrc
# ROS jazzy's setup.bash (sourced above) sets GZ_CONFIG_PATH to only the
# ROS-vendored gz_transport/gz_msgs/etc paths, which replaces rather than
# extends the CLI's built-in default of /usr/share/gz. That default is where
# the apt-installed gz-sim8/gz-gui8 packages register the "sim"/"gui" verbs,
# so once ROS's setup.bash has run, `gz sim` silently disappears from
# `gz --commands` and the Gazebo GUI never launches. Re-add it after ROS's
# setup so gz-tools can still find those verbs.
grep -q "GZ_CONFIG_PATH" /etc/bash.bashrc || \
    echo 'export GZ_CONFIG_PATH="${GZ_CONFIG_PATH}:/usr/share/gz"' >> /etc/bash.bashrc
grep -q "alias spawn_sphere=" /etc/bash.bashrc || \
    echo "alias spawn_sphere='/workspace/Perception/scripts/spawn_sphere.sh'" >> /etc/bash.bashrc
grep -q "alias perceive=" /etc/bash.bashrc || \
    echo "alias perceive='/workspace/Perception/scripts/run_perception_demo.sh'" >> /etc/bash.bashrc
grep -q "alias track=" /etc/bash.bashrc || \
    echo "alias track='/workspace/scripts/track'" >> /etc/bash.bashrc
