#!/bin/bash
# Brings up the in-container desktop that Gazebo renders into and Selkies
# streams out of. Idempotent — safe to re-run from VS Code's
# postStartCommand every time the container (re)starts.
#
#   Xvfb :0  ──►  openbox  ──►  selkies (WebSocket stream on :8080)
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
# Exact process-name match: a `pgrep -f` would also hit any shell whose
# command line merely mentions openbox (e.g. a one-liner that runs this).
if ! pgrep -x openbox >/dev/null 2>&1; then
    openbox >"$LOG/openbox.log" 2>&1 &
fi

# --- 3. Selkies ---------------------------------------------------------------
# Selkies 2.x attaches to the Xvfb above and streams it over a plain
# WebSocket on SELKIES_PORT. One port, so the VS Code forward is all the
# browser needs — no TURN relay. SELKIES_ENCODER=h264enc (compose env)
# lands on NVENC when the GPU is visible; the log's first line says so.
# Audio stays off (project decision); basic-auth user/password come from
# the compose env (initialize.sh generated the password into .env).
if ! running 'bin/selkies( |$)'; then
    export SELKIES_AUDIO_ENABLED=false
    export SELKIES_GAMEPAD_ENABLED=false
    export SELKIES_ENABLE_RESIZE="${SELKIES_ENABLE_RESIZE:-true}"
    export SELKIES_ENABLE_HTTPS=false
    export SELKIES_FRAMERATE="${SELKIES_FRAMERATE:-60}"
    export SELKIES_VIDEO_BITRATE="${SELKIES_VIDEO_BITRATE:-8000}"
    # Only the SSH tunnel can reach this, but the page is served to a
    # browser on a laptop, so bind to all container interfaces for the
    # forwarder to connect to (--public; the default is loopback only).
    selkies --public >"$LOG/selkies.log" 2>&1 &
fi

echo "desktop up: Xvfb ${DISPLAY}, Selkies on :${SELKIES_PORT:-8080} (logs in $LOG)"
