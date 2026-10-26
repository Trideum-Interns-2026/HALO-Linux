#!/bin/bash
# Runs ON THE SPARK HOST (devcontainer `initializeCommand`) before compose
# starts, from the developer's clone. Three jobs:
#   1. Make sure this clone knows whose it is (HALO_DEV in .env) — on a
#      shared login $USER is the same for everyone, so this is the only
#      thing that keeps five people's containers apart.
#   2. Generate a per-developer Selkies password on first use.
#   3. Build the unchanged root Dockerfile as halo-base:<dev> (the Spark
#      Dockerfile layers on it) and create the shared inference network.
set -euo pipefail
cd "$(dirname "$0")/../.."

ENV_FILE=.env

if [ ! -f "$ENV_FILE" ] || ! grep -q '^HALO_DEV=' "$ENV_FILE"; then
    cat >&2 <<'EOF'
error: HALO_DEV is not set.

Every developer's clone on the Spark needs its own identity, since all of us
log in as the same Linux user. Create a .env file in the repo root:

    echo "HALO_DEV=<yourname>" > .env        # lowercase, letters/digits/- only

then "Reopen in Container" again. See docs/spark/README.md.
EOF
    exit 1
fi

# shellcheck disable=SC1090
set -a; . "./$ENV_FILE"; set +a

if ! [[ "$HALO_DEV" =~ ^[a-z0-9][a-z0-9-]*$ ]]; then
    echo "error: HALO_DEV='$HALO_DEV' must be lowercase letters/digits/hyphens (it becomes a docker project name)" >&2
    exit 1
fi

# Dev Containers derives the compose project name from the folder name
# (identical for every clone) unless this is set — it's what actually keeps
# halo-cohen and halo-brooks apart when opened from VS Code.
grep -q '^COMPOSE_PROJECT_NAME=' "$ENV_FILE" || echo "COMPOSE_PROJECT_NAME=halo-${HALO_DEV}" >> "$ENV_FILE"

if ! grep -q '^SELKIES_BASIC_AUTH_PASSWORD=' "$ENV_FILE"; then
    pw="$(head -c 24 /dev/urandom | base64 | tr -dc 'a-zA-Z0-9' | head -c 20)"
    echo "SELKIES_BASIC_AUTH_PASSWORD=$pw" >> "$ENV_FILE"
    echo "generated a Selkies desktop password for '$HALO_DEV' (saved in .env)"
fi

docker network inspect halo-inference >/dev/null 2>&1 \
    || docker network create halo-inference >/dev/null

# Root Dockerfile, untouched. Layer cache makes this a no-op after the
# first developer builds it; the tag is per-dev only so one person's
# rebuild can't swap the base under another's running container.
echo "building halo-base:${HALO_DEV} from the root Dockerfile..."
docker build -t "halo-base:${HALO_DEV}" .
