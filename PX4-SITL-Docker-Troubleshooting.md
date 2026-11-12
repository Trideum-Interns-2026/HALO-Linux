# PX4 SITL Docker Setup — Troubleshooting Guide

This covers the Docker-based PX4 SITL + Gazebo setup and how to diagnose the
common failures.

## Prerequisites (one-time, on the host)

- Docker Desktop running (WSL2 backend enabled)
- WSLg available in the WSL2 distribution (Gazebo window)
- QGroundControl installed. WSL2: UDP ground-station link on port 14550.
  DGX Spark: TCP `127.0.0.1:5760` through the MAVLink TCP bridge (see `docs/spark/README.md`).

## The basic workflow

Build the image from the repo root (where the `Dockerfile` and `docker-compose*.yml` files are):

```
docker compose build
```

Then, inside the dev container (VS Code: **Dev Containers: Rebuild and Reopen in Container**):

```
fly                 # PX4 SITL + Gazebo; waits until /world/default/create exists
HEADLESS=1 fly      # same, without the Gazebo window
simlinks            # run next to fly: GPS bridge (/gps/fix) + uXRCE-DDS agent (/fmu/...)
```

`fly` runs the vehicle model `PX4_SIM_MODEL=gz_x500_depth` (X500 + OakD-Lite).
`fly` does not arm or take off.

MAVSDK connects on `udpin://0.0.0.0:14540`. Ground-station MAVLink is on UDP 14550.

---

## Common problems and fixes

### Gazebo does not start, or `fly` waits forever
`fly` waits until `/world/default/create` exists. The world name is usually
`default`, but scripts find it dynamically through `Perception/scripts/gz_world.sh`.
To list Gazebo image topics: `gz topic -l | grep image`.
Check that `fly` is running in its own terminal and has not exited.

### No camera images (`/camera/front/image_raw` missing)
The vehicle model must be `gz_x500_depth`. Plain `gz_x500` has no camera.
Check with `ros2 topic list`. The color camera is 1920x1080 at 30 Hz (the model
default). Do not cap it to 15 Hz.

### `perceive` finds nothing, or the detector does not start
The detector calls Triton (`HALO_TRITON_URL=triton:8001`), which is available
only on the Spark profile (`.devcontainer/spark/`). The WSL2 profile has no Triton.
Check `/perception/detection`: it publishes only when a person is seen.

### QGroundControl shows no vehicle
Confirm QGroundControl uses the link for your profile: UDP 14550 on WSL2, or
TCP `127.0.0.1:5760` on the Spark through the MAVLink TCP bridge. MAVSDK
(`udpin://0.0.0.0:14540`) is not a QGroundControl link.

### Spark: cannot see your topics, or see another developer's sim
Spark containers are fenced to localhost: `ROS_LOCALHOST_ONLY=1`,
`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, `GZ_IP=127.0.0.1`. Never remove them.
Check with `printenv | grep -E 'ROS_LOCALHOST|ROS_AUTOMATIC|GZ_IP'`.

### `movement` does nothing, or `tracking` / `geolocation` show no data
`fly`, `tracking`, `geolocation` and `movement` do not arm or take off. `movement`
needs an armed, airborne vehicle. Arm and take off in QGroundControl, or with
`sitl_vehicle arm` and `sitl_vehicle takeoff [m]`. Movement returns PX4 to Hold
on exit.

### "ROS_LOCALHOST_ONLY is deprecated" at node startup
Expected. Ignore it.

### Something from an earlier run is still running
Check with `simstack status`, then stop it with `simstack down`. `simstack` runs
the whole stack and never arms the vehicle.

### Image or dependencies look out of date
Rebuild from the repo root on the WSL2 host:
```
docker compose build --no-cache
```
