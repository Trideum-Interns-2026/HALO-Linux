# PX4 SITL Docker Setup — Troubleshooting Guide provided by Claude

This covers the Docker-based PX4 SITL + Gazebo setup and the issues we hit
getting it running, so the next person doesn't have to rediscover all of this.

## Prerequisites (one-time, on the host)

- Docker Desktop running (WSL2 backend enabled)
- VcXsrv running: Multiple windows, Display number `0`, Disable access control
- QGroundControl installed and open

## The basic workflow

From the folder containing `Dockerfile`, `docker-compose.yml`, and `entrypoint.sh`:

```
docker compose build
docker compose run --rm px4-sitl
```

First run clones PX4-Autopilot into a persistent Docker volume and builds it
from scratch — this takes several minutes. Every run after that reuses the
volume, so it launches quickly.

To get a shell instead of launching the sim:
```
docker compose run --rm px4-sitl bash
```

To launch a different vehicle model:
```
docker compose run --rm px4-sitl <target_name>
```

---

## Issues we ran into, and the fixes

### "no configuration file provided: not found"
You're not in the folder that contains `docker-compose.yml`. `cd` into it
first. Also double check the file isn't accidentally saved as
`docker-compose.yml.txt` (turn on file extensions in Windows Explorer to check).

### "failed to compute cache key ... entrypoint.sh not found"
`entrypoint.sh` needs to sit in the same folder as the `Dockerfile`. Same
`.txt` extension gotcha as above can cause this too.

### "exec /usr/local/bin/entrypoint.sh: exec format error"
Usually means `entrypoint.sh` is missing its `#!/bin/bash` shebang line on
line 1, or has Windows-style CRLF line endings / a BOM. Our Dockerfile now
strips CRLF/BOM automatically at build time, but the shebang line itself
must be present in the file as saved.

### CLI stuck printing "transferring context"
The Docker build context was huge (a full PX4-Autopilot checkout with `.git`
history). A `.dockerignore` fixes this by excluding files Docker doesn't
need to see. (Note: once we switched to cloning PX4-Autopilot *inside* the
container into a volume instead of copying a local folder in, this stopped
being an issue entirely — no more huge local folder in the build context.)

### "make: *** No rule to make target 'px4_sitl'. Stop."
The PX4 Makefile only defines `px4_sitl` as a target if it finds
`boards/px4/sitl/default.px4board` in the source tree — meaning the source
was incomplete (bad/partial predownload, or missing submodules).
**Fix:** use a genuinely fresh `git clone --recurse-submodules`.

### "ERROR [init] param import failed" / "px4-param: not found" / "no autostart file found"
This means the `build/` folder that got used was **stale or partially
built** — usually inherited from a predownloaded copy of PX4-Autopilot that
already had a `build/` folder in it from a previous, incomplete build.
**Fix:** `rm -rf build` inside the container, then rebuild from scratch.

### "ninja: error: unknown target '/bin/sh'" (or similar "unknown target" errors)
This happened even after a clean `rm -rf build`. The actual fix was running
a full **`make distclean`** instead — this is more thorough than deleting
`build/`, because it also resets git submodule state (NuttX, the
gazebo-classic sim submodule, etc.) that can get left in a broken state.

```
docker compose run --rm px4-sitl bash
make distclean
make px4_sitl gazebo-classic_typhoon_h480
```

If this keeps happening, it may be worth pinning PX4-Autopilot to a specific
release tag that matches the base Docker image's era, rather than always
building the bleeding-edge `main` branch against an older dev image.

### QGroundControl shows "Disconnected" even though a UDP link shows "Connected"
This is a networking issue, not a QGC settings issue. By default, PX4 SITL
sends its MAVLink heartbeats to its own container's `127.0.0.1` — not to
the Windows host — so under normal Docker bridge networking those packets
never actually reach QGroundControl.
**Fix:** enable Host Networking in Docker Desktop (Settings → Resources →
Network) and set `network_mode: host` in `docker-compose.yml`. This makes
the container share the host's network stack directly.

### Volume / network "Resource is still in use"
Something (usually a stopped container you forgot to `--rm`) is still
attached. Run `docker ps -a`, find it, `docker rm -f <name>`, then retry
`docker compose down -v`.

### Want to start completely fresh?
```
docker compose down -v
docker compose build --no-cache
docker compose run --rm px4-sitl
```
This wipes the persistent volume (source + build artifacts) and rebuilds
everything from scratch. Only do this if something's genuinely broken —
normal day-to-day use should never need it.
