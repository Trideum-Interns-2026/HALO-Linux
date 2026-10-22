#!/bin/bash
# Brings up the in-container desktop that Gazebo renders into and Selkies
# streams out of, then the ground station on it. Idempotent — VS Code runs
# it as postStartCommand every time the container (re)starts, and you can
# re-run it by hand (`desktop`) if anything died.
#
#   Xvfb :0  ──►  openbox  ──►  selkies (WebSocket on :8080)  ──►  QGroundControl
#
# Every daemon is started under setsid, in its own session. VS Code tears
# down postStartCommand's process group as soon as this script returns,
# which used to take Selkies (and anything else started with a plain `&`)
# with it — the Selkies tab then just spun forever. Dead children are
# reaped by `init: true` in docker-compose.spark.yml, so nothing lingers
# as a zombie and blocks a relaunch.
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
PORT="${SELKIES_PORT:-8080}"

# daemon <log name> <command...>: detached, logging to $LOG/<name>.log
daemon() {
    local name=$1; shift
    setsid "$@" >"$LOG/$name.log" 2>&1 </dev/null &
}

# --- 1. Virtual X server --------------------------------------------------
if ! pgrep -x Xvfb >/dev/null 2>&1; then
    echo "starting Xvfb on ${DISPLAY} (${RESOLUTION})"
    daemon xvfb Xvfb "${DISPLAY}" -screen 0 "${RESOLUTION}x24" +extension GLX \
        +extension RANDR +extension RENDER -nolisten tcp -noreset -dpi 96
    for _ in $(seq 1 50); do xdpyinfo >/dev/null 2>&1 && break; sleep 0.1; done
fi

# --- 2. Window manager ------------------------------------------------------
if ! pgrep -x openbox >/dev/null 2>&1; then
    daemon openbox openbox
fi

# --- 3. Selkies ---------------------------------------------------------------
# Selkies 2.x attaches to the Xvfb above and streams it over a plain
# WebSocket on SELKIES_PORT. One port, so the VS Code forward is all the
# browser needs — no TURN relay. SELKIES_ENCODER=h264enc (compose env)
# lands on NVENC when the GPU is visible; the log's first line says so.
# Audio stays off (project decision); basic-auth user/password come from
# the compose env (initialize.sh generated the password into .env).
if ! pgrep -f 'bin/selkies( |$)' >/dev/null 2>&1; then
    export SELKIES_AUDIO_ENABLED=false
    export SELKIES_GAMEPAD_ENABLED=false
    export SELKIES_ENABLE_RESIZE="${SELKIES_ENABLE_RESIZE:-true}"
    export SELKIES_ENABLE_HTTPS=false
    export SELKIES_FRAMERATE="${SELKIES_FRAMERATE:-60}"
    export SELKIES_VIDEO_BITRATE="${SELKIES_VIDEO_BITRATE:-8000}"
    # Only the SSH tunnel can reach this, but the page is served to a
    # browser on a laptop, so bind to all container interfaces for the
    # forwarder to connect to (--public; the default is loopback only).
    daemon selkies selkies --public
fi

# Don't return before the page answers: VS Code opens the browser tab on
# the 8080 forward as soon as the container is up, and a tab opened on a
# port nobody listens on yet is the "infinitely loading" Selkies page.
for _ in $(seq 1 100); do
    code=$(curl -s -o /dev/null -w '%{http_code}' --max-time 1 "http://127.0.0.1:${PORT}/" 2>/dev/null)
    [ "${code:-000}" != "000" ] && break
    sleep 0.2
done
if [ "${code:-000}" = "000" ]; then
    echo "warning: Selkies is not answering on :${PORT} yet — see $LOG/selkies.log" >&2
fi

# --- 4. QGroundControl --------------------------------------------------------
# Ground station on the same desktop (see /usr/local/bin/qgc for why it
# lives in here and not on the laptop). Step 1 of the README's "Running
# it" — "Open QGroundControl" — is therefore already done when you connect.
/usr/local/bin/qgc || true

echo "desktop up: Xvfb ${DISPLAY}, Selkies on :${PORT}, QGroundControl (logs in $LOG)"
