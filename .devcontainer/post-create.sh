#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash
if [ -f /opt/px4_ros2/install/setup.bash ]; then
    source /opt/px4_ros2/install/setup.bash
fi

mkdir -p /opt/ros2_ws/src
ln -sfn /workspace/perception_msgs /opt/ros2_ws/src/perception_msgs
ln -sfn /workspace/Perception /opt/ros2_ws/src/Perception
ln -sfn /workspace/geolocation_msgs /opt/ros2_ws/src/geolocation_msgs
ln -sfn /workspace/Geolocation /opt/ros2_ws/src/Geolocation
ln -sfn /workspace/tracking_msgs /opt/ros2_ws/src/tracking_msgs
ln -sfn /workspace/Tracking /opt/ros2_ws/src/Tracking
ln -sfn /workspace/Movement /opt/ros2_ws/src/Movement
ln -sfn /workspace/UI /opt/ros2_ws/src/UI
(cd /opt/ros2_ws && colcon build \
    --packages-select perception_msgs perception geolocation_msgs geolocation \
        tracking_msgs tracking movement ui \
    --symlink-install)

ln -sfn /workspace/Tracking/scripts/tracking /usr/local/bin/tracking
ln -sfn /workspace/Geolocation/scripts/geolocation /usr/local/bin/geolocation

# The bind-mounted repo is owned by the host user (uid 1000) but the container
# runs as root, so git refuses it as "dubious ownership" without this.
git config --global --get-all safe.directory | grep -qx /workspace || \
    git config --global --add safe.directory /workspace

# Make `fly`, ROS 2, PX4 messages, and the workspace overlay available in
# every new terminal VS Code opens.
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
sed -i '/^alias fly=/d' /etc/bash.bashrc
echo "alias fly='/workspace/scripts/fly'" >> /etc/bash.bashrc
grep -q "source /opt/ros/jazzy/setup.bash" /etc/bash.bashrc || \
    echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc
grep -q "ros2_ws/install/setup.bash" /etc/bash.bashrc || \
    echo "[ -f /opt/ros2_ws/install/setup.bash ] && source /opt/ros2_ws/install/setup.bash" >> /etc/bash.bashrc
sed -i '\#px4_ros2/install/setup.bash#d' /etc/bash.bashrc
sed -i '/ros2_ws\/install\/setup.bash/i [ -f /opt/px4_ros2/install/setup.bash ] \&\& source /opt/px4_ros2/install/setup.bash' /etc/bash.bashrc
# ROS jazzy's setup.bash (sourced above) sets GZ_CONFIG_PATH to only the
# ROS-vendored gz_transport/gz_msgs/etc paths, which replaces rather than
# extends the CLI's built-in default of /usr/share/gz. That default is where
# the apt-installed gz-sim8/gz-gui8 packages register the "sim"/"gui" verbs,
# so once ROS's setup.bash has run, `gz sim` silently disappears from
# `gz --commands` and the Gazebo GUI never launches. Re-add it after ROS's
# setup so gz-tools can still find those verbs.
grep -q "GZ_CONFIG_PATH" /etc/bash.bashrc || \
    echo 'export GZ_CONFIG_PATH="${GZ_CONFIG_PATH}:/usr/share/gz"' >> /etc/bash.bashrc
# Drop aliases for scripts that no longer exist, so they can't linger in a reused container.
sed -i '/^alias spawn_sphere=/d; /^alias track=/d' /etc/bash.bashrc
sed -i '/^alias spawn_person=/d' /etc/bash.bashrc
echo "alias spawn_person='/workspace/Perception/scripts/spawn_person.sh'" >> /etc/bash.bashrc
# `perceive` starts the camera bridge and YOLO detector (publishes /perception/detection).
sed -i '/^alias perceive=/d' /etc/bash.bashrc
echo "alias perceive='ros2 launch perception perception.launch.py'" >> /etc/bash.bashrc
sed -i '/^alias simlinks=/d' /etc/bash.bashrc
echo "alias simlinks='/workspace/scripts/sim_links'" >> /etc/bash.bashrc
sed -i '/^alias sim_check=/d' /etc/bash.bashrc
echo "alias sim_check='/workspace/scripts/sim_check'" >> /etc/bash.bashrc
sed -i '/^alias sitl_vehicle=/d' /etc/bash.bashrc
echo "alias sitl_vehicle='/workspace/scripts/sitl_vehicle'" >> /etc/bash.bashrc
sed -i '/^alias gps=/d' /etc/bash.bashrc
sed -i '/^alias tracking=/d' /etc/bash.bashrc
sed -i '/^alias geolocation=/d' /etc/bash.bashrc
echo "alias tracking='ros2 launch tracking tracking.launch.py'" >> /etc/bash.bashrc
echo "alias geolocation='ros2 launch geolocation geolocation.launch.py'" >> /etc/bash.bashrc
sed -i '/^alias ui=/d' /etc/bash.bashrc
echo "alias ui='ros2 launch ui ui.launch.py'" >> /etc/bash.bashrc
# `movement` = the movement service (follows the person over MAVLink offboard velocity)
sed -i '/^alias movement=/d' /etc/bash.bashrc
echo "alias movement='ros2 launch movement movement.launch.py'" >> /etc/bash.bashrc
# Whole sim stack in one command (up/down/status; never arms), the web dashboard, and all tests.
sed -i '/^alias simstack=/d; /^alias groundstation=/d; /^alias halo_test=/d' /etc/bash.bashrc
echo "alias simstack='/workspace/scripts/sim_stack'" >> /etc/bash.bashrc
echo "alias groundstation='python3 -m uvicorn main:app --app-dir /workspace/GroundstationAPI --host 127.0.0.1 --port 8000'" >> /etc/bash.bashrc
echo "alias halo_test='/workspace/scripts/halo_test'" >> /etc/bash.bashrc
# Web API dependencies (FastAPI + uvicorn) for `groundstation`; pip-only, no rosdep key.
python3 -m pip install -q --break-system-packages -r /workspace/GroundstationAPI/requirements.txt \
    || echo "WARNING: could not install GroundstationAPI requirements (offline?)"
