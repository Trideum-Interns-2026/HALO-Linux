#!/bin/bash
# Spark-profile postCreateCommand. Runs the shared post-create script
# unchanged, then adds what only the headless/Selkies setup needs.
set -e

bash /workspace/.devcontainer/post-create.sh

add_bashrc() { grep -qF "$1" /etc/bash.bashrc || echo "$1" >> /etc/bash.bashrc; }

# Gazebo renders into the Selkies virtual display.
add_bashrc 'export DISPLAY=:0'

# Route Gazebo's OpenGL through VirtualGL → NVIDIA EGL. Without this, Xvfb
# hands Gazebo llvmpipe (software GL) and the sim crawls. `fly` launches
# Gazebo and `perceive` opens the debug viewer, so both are wrapped;
# VGL_DISPLAY=egl is also exported for anything run with a bare `vglrun`.
add_bashrc 'export VGL_DISPLAY=egl'
add_bashrc 'alias fly="vglrun -d egl /workspace/scripts/fly"'
add_bashrc 'alias perceive="vglrun -d egl /workspace/Perception/scripts/run_perception_demo.sh"'
# The `fly` alias in the shared script was written as a plain path — the
# vglrun version above must come after it so it wins.
sed -i '/^alias fly=.\/workspace\/scripts\/fly.$/d' /etc/bash.bashrc

# Re-run the desktop bring-up by hand (idempotent) if Selkies or QGC died.
add_bashrc 'alias desktop="bash /usr/local/bin/start-desktop.sh"'

cat <<'EOF'

Spark container ready.
  - Desktop:   VS Code → Ports tab → "Selkies desktop" (user = HALO_DEV,
               password = SELKIES_BASIC_AUTH_PASSWORD from your .env)
               QGroundControl is already open there; `qgc` brings it back
               if you close it, `desktop` restarts the whole desktop.
  - Inference: HALO_TRITON_URL=$HALO_TRITON_URL (shared Triton)
  - Run `fly` / `perceive` as usual; they're wrapped in vglrun for GPU GL.
EOF
