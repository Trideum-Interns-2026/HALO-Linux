# PX4 SITL + Gazebo Classic dev container — amd64 only (Windows/WSL2)
#
# Uses PX4's own official px4io/px4-dev-ros2-foxy dev image, which ships
# ROS 2 Foxy, gazebo-classic, and PX4's build toolchain prebaked and
# tested together — much faster and more reliable than manually installing
# ROS 2 + Gazebo + build deps by hand on a bare Ubuntu base.
#
# NOTE: this image is amd64-only, so this Dockerfile no longer supports
# the ARM64 DGX Spark. Revisit docker-compose.spark.yml / the platform-
# agnostic Dockerfile from earlier in this project if that's needed again.

FROM px4io/px4-dev-ros2-foxy

# Set explicitly (not just sourced in .bashrc) so it's visible to any
# process that attaches to the container, including the VS Code ROS
# extension's non-interactive shell — otherwise it can't auto-detect the
# distro ("unable to determine ROS 2 distro").
ENV ROS_DISTRO=foxy
ENV ROS_LOCALHOST_ONLY=1

RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    && rm -rf /var/lib/apt/lists/*

# The base image's packages.ros.org signing key has since expired
# (upstream ROS apt-key rotation, unrelated to this repo) — apt-get update
# fails on that repo with EXPKEYSIG until it's refreshed. Fetch the current
# keyring, then point every ros*/ros2* sources.list.d entry at it via
# signed-by (works whether the image already used signed-by with a
# different path, or the older unqualified apt-key/trusted.gpg.d form).
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && for f in /etc/apt/sources.list.d/*ros*; do \
        grep -q 'signed-by=/usr/share/keyrings/ros-archive-keyring.gpg' "$f" || \
        sed -i -E 's#^deb (\[[^]]*\] *)?#deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] #' "$f"; \
    done

# gazebo_ros_pkgs (camera/init/factory plugins) + cv_bridge/OpenCV — needed
# for the Perception package's camera bridge (see
# Perception/scripts/patch_gazebo_camera_bridge.py). Not part of the base
# px4io/px4-dev-ros2-foxy image, which only ships gazebo-classic itself.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-foxy-gazebo-ros-pkgs \
    ros-foxy-cv-bridge \
    ros-foxy-vision-opencv \
    python3-opencv \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# MAVSDK C++ SDK — installed as a prebuilt .deb so your movement/test code
# can be built and run directly inside this container via CMake, instead
# of a separate throwaway Docker image per test run.
RUN curl -fL -o /tmp/mavsdk.deb \
       "https://github.com/mavlink/MAVSDK/releases/download/v3.17.4/libmavsdk-dev_3.17.4_ubuntu20.04_amd64.deb" \
    && apt-get update -o Acquire::AllowInsecureRepositories=true \
    && (dpkg -i /tmp/mavsdk.deb || apt-get install -f -y --allow-unauthenticated) \
    && rm -f /tmp/mavsdk.deb \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /PX4-Autopilot

EXPOSE 14540/udp
EXPOSE 14550/udp

COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN sed -i '1s/^\xEF\xBB\xBF//' /usr/local/bin/entrypoint.sh \
    && sed -i 's/\r$//' /usr/local/bin/entrypoint.sh \
    && chmod +x /usr/local/bin/entrypoint.sh

# Declared here, right before use, so changing this value only invalidates
# this cheap final layer — not the expensive layers above.
ARG PX4_TARGET=gazebo-classic_typhoon_h480
ENV PX4_TARGET=${PX4_TARGET}
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD []