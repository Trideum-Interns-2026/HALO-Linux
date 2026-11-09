---
name: px4-sitl-debugging
description: Load when the sim, Gazebo, PX4 or Triton misbehaves, to match the symptom to a known fix.
---
First read AGENTS.md "Environments", especially "Facts that cause bugs if forgotten" (depth model,
camera size, localhost fencing, world name). Then match the symptom. Diagnose and report; do not start
the sim yourself unless you are the sim-operator.

| Symptom | Likely cause and check |
| --- | --- |
| `no configuration file provided: not found` | Not in the folder with `docker-compose.yml`, or it is saved as `.yml.txt` |
| `entrypoint.sh not found` / `exec format error` | File missing, wrong folder, or no shebang / CRLF line endings |
| `No rule to make target 'px4_sitl'` | Incomplete PX4 source; needs a fresh `git clone --recurse-submodules` |
| `param import failed`, `no autostart file found` | Stale or partial `build/`; remove it and rebuild |
| `ninja: unknown target` after cleaning `build/` | `make distclean` (also resets submodules) |
| QGroundControl `Disconnected` though UDP shows connected | Heartbeats stay inside the container; host networking (WSL2 profile) |
| Spark: QGroundControl no vehicle | Link must be TCP `127.0.0.1:5760`; 5760 in the Ports tab; `fly` running; `cat /tmp/desktop/mavlink-bridge.log`; `desktop` restarts the bridge |
| Spark: Selkies page spins or never loads | Nothing listening on 8080 in your container; run `desktop`; forward from the container window, not the host one |
| Spark: Selkies black or connecting | `cat /tmp/desktop/selkies.log`, first line should say `nvenc` |
| Gazebo slow, `glxinfo -B` says llvmpipe | Run under `vglrun -d egl` |
| Detector logs `skipped N (inference unavailable)` | Triton down, or the dev container is not on `halo-inference` |
| Someone else's drone in your sim | Same `HALO_DEV` in two `.env` files |
| No camera topic | Model is not `gz_x500_depth`, or the world is not `default`; `gz topic -l \| grep image` |
| `ROS_LOCALHOST_ONLY is deprecated` | Expected; ignore |

Full text: `PX4-SITL-Docker-Troubleshooting.md` and `docs/spark/README.md` (Troubleshooting). On the
Spark host, `scripts/spark/verify.sh` checks the whole chain.
