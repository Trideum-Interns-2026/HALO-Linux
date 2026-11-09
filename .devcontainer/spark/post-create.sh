#!/bin/bash
# Spark-profile postCreateCommand. Runs the shared post-create script
# unchanged, then adds what only the headless/Selkies setup needs.
set -e

bash /workspace/.devcontainer/post-create.sh

add_bashrc() { grep -qF "$1" /etc/bash.bashrc || echo "$1" >> /etc/bash.bashrc; }

# Gazebo renders into the Selkies virtual display.
add_bashrc 'export DISPLAY=:0'

# Route Gazebo's OpenGL through VirtualGL → NVIDIA EGL. Without this, Xvfb
# hands Gazebo llvmpipe (software GL) and the sim crawls. Only `fly` opens
# a window, so only it is wrapped (`perceive` is now a plain ros2 launch
# with no GUI). VGL_DISPLAY=egl is also exported for a bare `vglrun`.
add_bashrc 'export VGL_DISPLAY=egl'
add_bashrc 'alias fly="vglrun -d egl /workspace/scripts/fly"'
# The `fly` alias in the shared script was written as a plain path — the
# vglrun version above must come after it so it wins.
sed -i '/^alias fly=.\/workspace\/scripts\/fly.$/d' /etc/bash.bashrc

# Re-run the desktop bring-up by hand (idempotent) if Selkies or the
# MAVLink bridge died.
add_bashrc 'alias desktop="bash /usr/local/bin/start-desktop.sh"'

cat <<'EOF'

Spark container ready.
  - Desktop:   VS Code → Ports tab → "Selkies desktop" (user = HALO_DEV,
               password = SELKIES_BASIC_AUTH_PASSWORD from your .env)
               `desktop` restarts it (and the MAVLink bridge) if needed.
  - QGC:       on YOUR LAPTOP — Comm Links → TCP → 127.0.0.1:5760
               (forwarded by VS Code like the desktop; see docs/spark).
  - Inference: HALO_TRITON_URL=$HALO_TRITON_URL (shared Triton)
  - Run `fly` as usual (wrapped in vglrun for GPU GL), then `perceive`.
EOF
