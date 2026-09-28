#!/bin/bash
# Sourced by the spawn scripts: sets WORLD to the name of the Gazebo world
# that's currently running (from its /world/<name>/create service), so they
# work in any world instead of assuming PX4's "default". Export WORLD
# yourself to override, e.g. if more than one world is running.
if [ -z "${WORLD:-}" ]; then
    WORLD=$(gz service -l 2>/dev/null | sed -n 's#^/world/\([^/]*\)/create$#\1#p' | head -n1)
fi
if [ -z "$WORLD" ]; then
    echo "No running Gazebo world found — is \`fly\` running?" >&2
    exit 1
fi
