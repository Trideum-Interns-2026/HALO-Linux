# Windows Setup: Repo → Ubuntu (WSL2) → Docker → Gazebo

This is the full path from a fresh Windows machine to having PX4 SITL +
Gazebo running with GPU acceleration, and a working dev container for
writing/building your own code (Movement, Tracking, Geolocation, etc.).

**Important:** everything in this guide happens inside WSL2's own Linux
filesystem, not your `C:\Users\...` folder. Cloning into the Windows
filesystem instead of WSL2's breaks GPU-accelerated rendering later on —
this is not optional.

---

## 1. Prerequisites (should already be set up)

This guide assumes WSL2 and Docker Desktop are already installed. Quick
checklist to confirm:

- **WSL2 + Ubuntu**: open the Start menu, confirm "Ubuntu" is there and
  opens to a Linux terminal. If not, run `wsl --install` in an
  Administrator PowerShell and restart.
- **Docker Desktop**: confirm it's installed and the whale icon shows in
  the system tray.
- **Settings → General**: "Use the WSL 2 based engine" is checked.
- **Settings → Resources → WSL Integration**: toggled **on** for your
  Ubuntu distro specifically (not just the "default distro" toggle — both
  need to be on). Restart Docker Desktop if you change this.
- **WSL2 memory limit** — without this, heavy builds can get killed by an
  out-of-memory error. Check `C:\Users\<you>\.wslconfig` exists with
  something like:
  ```ini
  [wsl2]
  memory=8GB
  ```
  Adjust based on your actual RAM. If you create/change this, run
  `wsl --shutdown` in PowerShell and reopen Docker Desktop afterward.

## 2. Install VS Code + extensions

Install VS Code normally on Windows (not inside WSL2). Then install these
extensions:
- **WSL** (Microsoft) — lets VS Code connect into WSL2
- **Dev Containers** (Microsoft) — the actual dev container support

## 3. Clone the repo — inside WSL2

Open the **Ubuntu** app (not PowerShell). Confirm `git` works:
```bash
git --version
```
If missing: `sudo apt update && sudo apt install -y git`

Clone into WSL2's own home folder (not `/mnt/c/...`):
```bash
cd ~
git clone https://github.com/Trideum-Interns-2026/HALO-Linux.git
cd HALO-Linux
```

## 4. Build the image

Still in the WSL2 terminal:
```bash
docker compose build
```
First build takes a few minutes (downloading the base PX4 dev image, MAVSDK,
etc.). Subsequent builds reuse Docker's layer cache and are much faster.

## 5. Open the dev container for development

This is the environment you actually write and build code in.

```bash
code .
```
Wait for VS Code to open — confirm the **bottom-left corner** shows a WSL
indicator before proceeding (if it doesn't, something's wrong; don't
continue until it does).

In VS Code: **Ctrl+Shift+P → "Dev Containers: Rebuild and Reopen in
Container"**. First time, this builds the dev container on top of the same
image. Wait for it to finish and attach.

You should now see your own repo files in the Explorer (`Movement/`,
`Tracking/`, `Geolocation/`, `Dockerfile`, etc.) — **not** PX4-Autopilot's
source tree. PX4's source lives in a separate volume, kept intentionally
out of your way.

## 6. Run the stack from inside the dev container

Open a terminal in VS Code (it's already inside the dev container). Run:
```bash
fly
```
This starts PX4 SITL and Gazebo. Leave this terminal running. `HEADLESS=1 fly`
runs without the Gazebo window. `fly` does not arm or take off.

Start the other pieces in their own terminals as needed:
- `simlinks`: sim only, run next to `fly`. GPS bridge to `/gps/fix` and the
  uXRCE-DDS agent (`/fmu/...`).
- `spawn_person [x] [y] [yaw]`: test target, default `5 0 0` (5 m ahead,
  facing the drone).
- `perceive`: camera bridge and person detector. The detector needs Triton,
  so this runs on the Spark profile only.
- `tracking`, `geolocation`: position services.
- `movement`: follows the person. Needs an armed, airborne vehicle.
- `ui`: UI bridge node. `groundstation`: web dashboard at
  http://127.0.0.1:8000 (port 8000 is forwarded by the dev container).

Arming and flight:
- Arm and take off in QGroundControl, or with `sitl_vehicle arm`,
  `sitl_vehicle takeoff [m]` (SITL only, takeoff up to 10 m), `sitl_vehicle land`,
  `sitl_vehicle disarm`, `sitl_vehicle status`.
- `sim_check [--gui|--watch]`: runs the whole stack, arms and flies it, prints
  a PASS/FAIL table, then tears down.
- `simstack up [--gui]`, `simstack status`, `simstack down`: whole stack in one
  command. It never arms the vehicle.

## 7. Run the tests

`halo_test` runs all tests for every service. Tests need no sim, GPU or Triton.

---

## If something goes wrong

Check these docs, in the same repo, for known issues and fixes:
- `PX4-SITL-Docker-Troubleshooting.md` — build/runtime issues with PX4 itself

A few of the most common gotchas, up front:
- **Opening the repo from the Windows path** (`C:\Users\...`) instead of
  WSL2's own filesystem breaks GPU-accelerated Gazebo rendering entirely.
  Always `code .` from inside the WSL2 terminal.
- **Running `docker` commands from inside the dev container** doesn't work —
  the dev container has no Docker-in-Docker setup. Run `docker compose` from
  your WSL2 host terminal. Inside the dev container, use `fly` and the other
  commands in section 6.
- **Git commands run from inside the dev container** can leave files
  owned by `root` on the host, causing permission errors later. Do git
  operations from your WSL2 terminal, not the dev container terminal.
