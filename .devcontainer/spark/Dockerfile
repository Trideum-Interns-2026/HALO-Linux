# Spark-only layer on top of the normal px4-sitl-dev image (root Dockerfile,
# which this file deliberately does NOT change). Adds what a headless,
# per-developer container on the DGX Spark needs:
#
#   1. A virtual X display + lightweight WM for Gazebo (Xvfb + openbox).
#   2. VirtualGL, so Gazebo's OpenGL goes to the NVIDIA GPU through EGL
#      instead of Xvfb's software renderer (the "egl-desktop" pattern from
#      selkies-project/docker-selkies-egl-desktop).
#   3. Selkies 2.x: streams that display to the browser, NVENC-encoded,
#      over a plain WebSocket on one port — so it rides the developer's
#      SSH port-forward directly (no WebRTC/TURN relay needed).
#   4. QGroundControl, running on that same desktop: with bridge networking
#      a QGC on the laptop never sees the sim's MAVLink, so the ground
#      station lives in here next to PX4 (autostarted by start-desktop.sh).
#   5. The Triton Python client for the shared YOLO inference service.
#
# Built by docker-compose.spark.yml; BASE_IMAGE is the root Dockerfile's
# output (halo-base:<dev>), built just before by initialize.sh.

ARG BASE_IMAGE=px4-sitl-dev:latest
ARG TRITON_TAG=25.09

# --- tritonclient wheel source --------------------------------------------
# A plain `pip install tritonclient` on ARM is known to resolve to the wrong
# (Jetson/iGPU) build. The Triton SDK image for the same release carries the
# correct aarch64 SBSA wheel, so take it from there instead.
FROM nvcr.io/nvidia/tritonserver:${TRITON_TAG}-py3-sdk AS tritonsdk

FROM ${BASE_IMAGE}
ARG TARGETARCH

# Selkies release (github.com/selkies-project/selkies). 2.x publishes one
# .deb per Ubuntu release/arch; the arm64 Ubuntu 24.04 one is what the DGX
# Spark needs. The 1.x "selkies-gstreamer" releases were removed upstream.
ARG SELKIES_VERSION=2.0.0
ARG VIRTUALGL_VERSION=3.1.2
ARG UBUNTU_RELEASE=24.04

ENV DEBIAN_FRONTEND=noninteractive

# X server, window manager, GL diagnostics. Selkies' own system libraries
# come in through its .deb's Depends below. No PulseAudio — audio streaming
# is deliberately disabled for this project.
RUN apt-get update && apt-get install -y --no-install-recommends \
        xvfb xauth x11-xserver-utils x11-utils xdotool \
        openbox \
        mesa-utils libegl1 libgl1 libglx-mesa0 libglvnd0 \
        libgles2 libopengl0 vulkan-tools \
        python3-pip python3-venv \
        xsel xclip \
    && rm -rf /var/lib/apt/lists/*

# VirtualGL. `vglrun -d egl <app>` renders through the NVIDIA EGL device
# and copies frames into Xvfb — no real Xorg needed. Arm64 .deb is a
# published release asset.
RUN case "$TARGETARCH" in \
        amd64) VGL_ARCH=amd64 ;; \
        arm64) VGL_ARCH=arm64 ;; \
        *) echo "Unsupported TARGETARCH: $TARGETARCH" >&2; exit 1 ;; \
    esac \
    && curl -fL -o /tmp/virtualgl.deb \
        "https://github.com/VirtualGL/virtualgl/releases/download/${VIRTUALGL_VERSION}/virtualgl_${VIRTUALGL_VERSION}_${VGL_ARCH}.deb" \
    && apt-get update && apt-get install -y --no-install-recommends /tmp/virtualgl.deb \
    && rm -f /tmp/virtualgl.deb && rm -rf /var/lib/apt/lists/* \
    && chmod u+s /usr/lib/libvglfaker.so /usr/lib/libdlfaker.so 2>/dev/null || true

# Selkies: the .deb carries the server, its web client and a private Python
# env under /opt/selkies, and puts `selkies` on PATH; apt resolves the
# system libraries it depends on. It attaches to the Xvfb that
# start-desktop.sh runs — it brings no display of its own.
RUN curl -fL -o /tmp/selkies.deb \
        "https://github.com/selkies-project/selkies/releases/download/${SELKIES_VERSION}/selkies-${SELKIES_VERSION}-ubuntu${UBUNTU_RELEASE}-${TARGETARCH}.deb" \
    && apt-get update && apt-get install -y --no-install-recommends /tmp/selkies.deb \
    && rm -f /tmp/selkies.deb && rm -rf /var/lib/apt/lists/*

# QGroundControl. Stable releases ship an x86_64 and an aarch64 AppImage.
# There's no FUSE in the container, so it's extracted here once (→
# /opt/qgc, run via /opt/qgc/AppRun) instead of self-extracting into /tmp
# on every launch (the extraction comes out root-only, hence the chmod).
# QGC refuses to run as root, so /usr/local/bin/qgc starts it as `ubuntu`
# (already in the base image; created if not) — its home is a compose
# volume so QGC's settings and map cache persist.
ARG QGC_VERSION=5.1.5
RUN set -e \
    && case "$TARGETARCH" in \
        amd64) QGC_ARCH=x86_64 ;; \
        arm64) QGC_ARCH=aarch64 ;; \
        *) echo "Unsupported TARGETARCH: $TARGETARCH" >&2; exit 1 ;; \
    esac \
    && cd /tmp \
    && curl -fL -o qgc.AppImage \
        "https://github.com/mavlink/qgroundcontrol/releases/download/v${QGC_VERSION}/QGroundControl-${QGC_ARCH}.AppImage" \
    && chmod +x qgc.AppImage \
    && ./qgc.AppImage --appimage-extract >/dev/null \
    && mv squashfs-root /opt/qgc \
    && chmod -R a+rX /opt/qgc \
    && rm -f qgc.AppImage \
    && (id ubuntu >/dev/null 2>&1 || useradd -m -s /bin/bash ubuntu) \
    && mkdir -p /home/ubuntu/.config /home/ubuntu/.cache \
    && chown -R ubuntu:ubuntu /home/ubuntu

# Triton gRPC client, from the SDK image's own aarch64/x86_64 wheel (see
# the tritonsdk stage above). The SDK ships two wheels — a pure-Python
# `py3-none-any` one and the `manylinux` platform one that carries the
# compiled gRPC bits — so pick the platform wheel explicitly. numpy<2 is
# re-pinned for the same reason as in the root Dockerfile — the apt ROS
# stack breaks on numpy 2.
COPY --from=tritonsdk /workspace/install/python/tritonclient-*.whl /tmp/
RUN WHL="$(ls /tmp/tritonclient-*manylinux*.whl | head -n1)" \
    && test -n "$WHL" \
    && pip3 install --no-cache-dir --break-system-packages "${WHL}[grpc]" "numpy<2" \
    && rm -f /tmp/tritonclient-*.whl

# Spark-only entry scripts. Kept outside /workspace so they work even
# before the bind mount is up.
COPY .devcontainer/spark/start-desktop.sh /usr/local/bin/start-desktop.sh
COPY .devcontainer/spark/qgc /usr/local/bin/qgc
RUN chmod +x /usr/local/bin/start-desktop.sh /usr/local/bin/qgc

ENV DISPLAY=:0
WORKDIR /workspace
