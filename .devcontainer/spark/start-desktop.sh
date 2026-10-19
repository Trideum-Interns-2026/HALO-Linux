#!/bin/bash
# Brings up the in-container desktop that Gazebo renders into and Selkies
# streams out of. Idempotent — safe to re-run from VS Code's
# postStartCommand every time the container (re)starts.
#
#   Xvfb :0  ──►  openbox  ──►  coturn (TCP relay)  ──►  selkies-gstreamer
#
# GPU OpenGL for Gazebo comes from VirtualGL (`vglrun -d egl`), not from
# Xvfb itself — see the `gz`/`fly` wrappers in post-create.sh. Xvfb is just
# the framebuffer the window manager and Selkies see.
#
# Logs: /tmp/desktop/*.log

set -u
LOG=/tmp/desktop
mkdir -p "$LOG"

DISPLAY_NUM="${DISPLAY#:}"
DISPLAY_NUM="${DISPLAY_NUM:-0}"
export DISPLAY=":${DISPLAY_NUM}"
# Keep in sync with the 1080p camera/Gazebo window sizes used elsewhere.
RESOLUTION="${SELKIES_RESOLUTION:-1920x1080}"

running() { pgrep -f "$1" >/dev/null 2>&1; }

# --- 1. Virtual X server --------------------------------------------------
if ! running "Xvfb ${DISPLAY}"; then
    echo "starting Xvfb on ${DISPLAY} (${RESOLUTION})"
    Xvfb "${DISPLAY}" -screen 0 "${RESOLUTION}x24" +extension GLX +extension RANDR \
        +extension RENDER -nolisten tcp -noreset -dpi 96 >"$LOG/xvfb.log" 2>&1 &
    for _ in $(seq 1 50); do xdpyinfo >/dev/null 2>&1 && break; sleep 0.1; done
fi

# --- 2. Window manager ------------------------------------------------------
if ! running "openbox"; then
    openbox >"$LOG/openbox.log" 2>&1 &
fi

# --- 3. TURN relay ------------------------------------------------------------
# WebRTC media can't ride the SSH port-forward on its own; coturn in TCP
# mode on 3478 gives the browser a relay it CAN reach through the forward.
# Long-term credentials match SELKIES_TURN_USERNAME/PASSWORD so both the
# browser and selkies-gstreamer authenticate to it the same way.
if ! running "turnserver"; then
    turnserver \
        --listening-port="${SELKIES_TURN_PORT:-3478}" \
        --listening-ip=0.0.0.0 \
        --relay-ip=127.0.0.1 \
        --external-ip=127.0.0.1 \
        --no-udp --no-dtls --no-tls \
        --lt-cred-mech \
        --user="${SELKIES_TURN_USERNAME}:${SELKIES_TURN_PASSWORD}" \
        --realm=halo.local \
        --no-cli --no-multicast-peers --fingerprint \
        --log-file=stdout >"$LOG/coturn.log" 2>&1 &
fi

# --- 4. Selkies-GStreamer -----------------------------------------------------
if ! running "selkies-gstreamer"; then
    # The release bundle ships its own GStreamer; this puts it on PATH and
    # the library paths. Audio stays off (project decision).
    # shellcheck disable=SC1091
    . /opt/gstreamer/gst-env
    export SELKIES_ENABLE_AUDIO=false
    export SELKIES_ENABLE_RESIZE="${SELKIES_ENABLE_RESIZE:-true}"
    export SELKIES_ENABLE_HTTPS=false
    # Only the SSH tunnel can reach this, but the page is served to a
    # browser on a laptop, so bind to all container interfaces for the
    # forwarder to connect to.
    export SELKIES_ADDR=0.0.0.0
    export SELKIES_WEB_ROOT=/opt/gst-web
    export SELKIES_FRAMERATE="${SELKIES_FRAMERATE:-60}"
    export SELKIES_VIDEO_BITRATE="${SELKIES_VIDEO_BITRATE:-8000}"
    selkies-gstreamer >"$LOG/selkies.log" 2>&1 &
fi

echo "desktop up: Xvfb ${DISPLAY}, Selkies on :${SELKIES_PORT:-8080} (logs in $LOG)"
