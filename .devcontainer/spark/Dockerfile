# Spark-only layer on top of the normal px4-sitl-dev image (root Dockerfile,
# which this file deliberately does NOT change). Adds what a headless,
# per-developer container on the DGX Spark needs:
#
#   1. A virtual X display + lightweight WM for Gazebo (Xvfb + openbox).
#   2. VirtualGL, so Gazebo's OpenGL goes to the NVIDIA GPU through EGL
#      instead of Xvfb's software renderer (the "egl-desktop" pattern from
#      selkies-project/docker-selkies-egl-desktop).
#   3. Selkies-GStreamer: WebRTC streaming of that display to the browser
#      with NVENC hardware encode, plus coturn for relaying the media
#      through the developer's SSH tunnel.
#   4. The Triton Python client for the shared YOLO inference service.
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

# Selkies-GStreamer release. The arm64 GStreamer bundle for Ubuntu 24.04 is
# what the DGX Spark needs — the verify script's `docker compose build`
# step is the test that this tag actually publishes that asset.
ARG SELKIES_VERSION=1.6.2
ARG VIRTUALGL_VERSION=3.1.2
ARG UBUNTU_RELEASE=24.04

ENV DEBIAN_FRONTEND=noninteractive

# X server, window manager, GL diagnostics, TURN server, and the runtime
# libraries the Selkies GStreamer bundle links against. No PulseAudio —
# audio streaming is deliberately disabled for this project.
RUN apt-get update && apt-get install -y --no-install-recommends \
        xvfb xauth x11-xserver-utils x11-utils xdotool \
        openbox \
        mesa-utils libegl1 libgl1 libglx-mesa0 libglvnd0 \
        libgles2 libopengl0 vulkan-tools \
        coturn \
        python3-pip python3-venv \
        libaa1 libaom3 libass9 libavcodec60 libavfilter9 libavformat60 \
        libavutil58 libcairo2 libdav1d7 libdrm2 libegl-mesa0 libgbm1 \
        libgdk-pixbuf-2.0-0 libgirepository-1.0-1 libgstreamer-gl1.0-0 \
        libgstreamer-plugins-base1.0-0 libgstreamer1.0-0 libgudev-1.0-0 \
        libjpeg-turbo8 libnice10 libopenh264-7 libopus0 libpango-1.0-0 \
        libpangocairo-1.0-0 libpng16-16 libsrtp2-1 libvpx9 libwebp7 \
        libwebrtc-audio-processing1 libx264-164 libx265-199 libxcb1 \
        libxcomposite1 libxcursor1 libxdamage1 libxext6 libxfixes3 \
        libxi6 libxkbcommon0 libxrandr2 libxrender1 libxtst6 libxv1 \
        gir1.2-gst-plugins-base-1.0 python3-gi python3-gi-cairo \
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

# Selkies-GStreamer: prebuilt GStreamer bundle (→ /opt/gstreamer), the
# Python package, and the web frontend (→ /opt/gst-web).
RUN set -e \
    && SELKIES_ARCH="$TARGETARCH" \
    && cd /tmp \
    && curl -fL -o gstreamer.tar.gz \
        "https://github.com/selkies-project/selkies-gstreamer/releases/download/v${SELKIES_VERSION}/gstreamer-selkies_gpl_v${SELKIES_VERSION}_ubuntu${UBUNTU_RELEASE}_${SELKIES_ARCH}.tar.gz" \
    && tar -xzf gstreamer.tar.gz -C /opt && rm gstreamer.tar.gz \
    && curl -fL -o selkies_gstreamer.whl \
        "https://github.com/selkies-project/selkies-gstreamer/releases/download/v${SELKIES_VERSION}/selkies_gstreamer-${SELKIES_VERSION}-py3-none-any.whl" \
    && pip3 install --no-cache-dir --break-system-packages ./selkies_gstreamer.whl \
    && rm selkies_gstreamer.whl \
    && curl -fL -o selkies-gstreamer-web.tgz \
        "https://github.com/selkies-project/selkies-gstreamer/releases/download/v${SELKIES_VERSION}/selkies-gstreamer-web_v${SELKIES_VERSION}.tgz" \
    && tar -xzf selkies-gstreamer-web.tgz -C /opt && rm selkies-gstreamer-web.tgz

# Triton gRPC client, from the SDK image's own aarch64/x86_64 wheel (see
# the tritonsdk stage above). numpy<2 is re-pinned for the same reason as
# in the root Dockerfile — the apt ROS stack breaks on numpy 2.
COPY --from=tritonsdk /workspace/install/python/tritonclient-*.whl /tmp/
RUN pip3 install --no-cache-dir --break-system-packages \
        "$(ls /tmp/tritonclient-*.whl)[grpc]" "numpy<2" \
    && rm -f /tmp/tritonclient-*.whl

# Spark-only entry scripts. Kept outside /workspace so they work even
# before the bind mount is up.
COPY .devcontainer/spark/start-desktop.sh /usr/local/bin/start-desktop.sh
RUN chmod +x /usr/local/bin/start-desktop.sh

ENV DISPLAY=:0
WORKDIR /workspace
