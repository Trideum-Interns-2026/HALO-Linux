FROM px4io/px4-sitl-gazebo-ros2:latest

# Set explicitly (not just sourced in .bashrc) so it's visible to any
# process that attaches to the container, including the VS Code ROS
# extension's non-interactive shell — otherwise it can't auto-detect the
# distro ("unable to determine ROS 2 distro").
ENV ROS_DISTRO=jazzy

# colcon + cv_bridge/OpenCV + ros_gz_bridge — needed to build and run the
# Perception ROS 2 package. Not included in this runtime-focused base image
# (it's meant to just run a prebuilt sim, not build/run ROS 2 vision code
# against it). Confirmed missing the hard way, 2026-09-24 — perception.launch.py
# failed with "ModuleNotFoundError: No module named 'cv2'" without this.
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-opencv \
    ros-jazzy-cv-bridge \
    ros-jazzy-vision-opencv \
    ros-jazzy-rqt-image-view \
    ros-jazzy-ros-gz-bridge \
    curl \
    && rm -rf /var/lib/apt/lists/*

# MAVSDK C++ SDK — installed as a prebuilt .deb so the Movement/
# Tracking_Geolocation C++ test programs (run_all.py) can be built and run
# directly inside this container via CMake. This base image is Ubuntu 24.04
# (Noble), not the 20.04 (Focal) the old Dockerfile targeted.
#
# TARGETARCH is set automatically by buildx (the default builder since
# Docker Engine 23 / current Docker Compose) to whatever arch is actually
# being built — amd64 on a normal PC, arm64 on the ARM64 NVIDIA DGX Spark —
# so the matching release asset is picked instead of always grabbing amd64.
# MAVSDK doesn't publish an ubuntu24.04 arm64 .deb (checked v3.17.1's
# GitHub release assets) — arm64 only gets Debian builds. debian12
# (Bookworm, glibc 2.36) is older than and ABI-compatible with Ubuntu
# 24.04's glibc 2.39, so it installs and runs fine here despite the label.
ARG TARGETARCH
RUN case "$TARGETARCH" in \
        amd64) MAVSDK_DEB="libmavsdk-dev_3.17.1_ubuntu24.04_amd64.deb" ;; \
        arm64) MAVSDK_DEB="libmavsdk-dev_3.17.1_debian12_arm64.deb" ;; \
        *) echo "Unsupported TARGETARCH: $TARGETARCH" >&2; exit 1 ;; \
    esac \
    && curl -fL -o /tmp/mavsdk.deb \
       "https://github.com/mavlink/MAVSDK/releases/download/v3.17.1/${MAVSDK_DEB}" \
    && apt-get update -o Acquire::AllowInsecureRepositories=true \
    && (dpkg -i /tmp/mavsdk.deb || apt-get install -f -y --allow-unauthenticated) \
    && rm -f /tmp/mavsdk.deb \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace