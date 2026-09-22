# Windows Setup: Repo → Ubuntu (WSL2) → Docker → Gazebo

This is the full path from a fresh Windows machine to having PX4 SITL +
Gazebo running with GPU acceleration, and a working dev container for
writing/building your own code (Movement, Tracking_Geolocation, etc.).

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
git clone https://github.com/Trideum-Interns-2026/HALO.git
cd HALO/Docker
```

## 4. Build the image

Still in the WSL2 terminal:
```bash
docker compose build
```
First build takes a few minutes (downloading the base PX4 dev image, MAVSDK,
etc.). Subsequent builds reuse Docker's layer cache and are much faster.

## 5. Quick check: launch the sim standalone

This is the fastest way to confirm everything works before touching VS Code:
```bash
python3 run.py
```
This clones PX4-Autopilot into a persistent volume (first run only, several
minutes) and launches Gazebo with the default vehicle model. A Gazebo window
should appear directly on your Windows desktop via WSLg — no extra X server
setup needed. Ctrl+C to stop it when you're done checking.

## 6. Open the dev container for development

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
`Tracking_Geolocation/`, `Dockerfile`, etc.) — **not** PX4-Autopilot's
source tree. PX4's source lives in a separate volume, kept intentionally
out of your way.

## 7. Run SITL + Gazebo from inside the dev container

Open a terminal in VS Code (it's already inside the dev container). Run:
```bash
fly
```
This is a shortcut for `cd /PX4-Autopilot && make px4_sitl gazebo-classic_typhoon_h480`.
Leave this terminal running.

## 8. Build and run your own code

`run_all.py` works from **either** the dev container or your WSL2 host
terminal — it auto-detects where it's running and adapts (direct
`cmake`/`make` inside the dev container, a throwaway Docker image if run
from the WSL2 host).

From the repository root:
```bash
python3 run_all.py <folder> <file.cpp>
```
The folder search is recursive, so this runs the file inside the
Geolocation subfolder:
```bash
python3 run_all.py Tracking_Geolocation takeoff_forward_back.cpp
```
For the Movement folder:
```bash
python3 run_all.py Movement test_takeoff.cpp
```
Either way, it connects straight to whatever SITL instance is currently
running (started via `run.py` or `fly`) — no extra config needed.

**Three things, worth keeping straight:**
- **`run.py`** — WSL2 host terminal only (not the dev container). Launches
  the standalone PX4 SITL + Gazebo container.
- **`fly`** — inside the dev container only. Launches SITL from within
  the dev container itself.
- **`run_all.py`** — works from either place. Builds and runs one of your
  C++ programs against whatever SITL instance is currently running.

---

## If something goes wrong

Check these docs, in the same repo, for known issues and fixes:
- `PX4-SITL-Docker-Troubleshooting.md` — build/runtime issues with PX4 itself
- `WSL-Git-Setup.md` — git permission/ownership issues specific to WSL2

A few of the most common gotchas, up front:
- **Opening the repo from the Windows path** (`C:\Users\...`) instead of
  WSL2's own filesystem breaks GPU-accelerated Gazebo rendering entirely.
  Always `code .` from inside the WSL2 terminal.
- **Running `run.py` or `run_all.py` from inside the dev container**
  doesn't work — both call `docker` directly, and the dev container has no
  Docker-in-Docker setup. Run both from your WSL2 host terminal. If you're
  already in the dev container and just want to launch SITL there, use
  `fly` instead.
- **Git commands run from inside the dev container** can leave files
  owned by `root` on the host, causing permission errors later. Do git
  operations from your WSL2 terminal, not the dev container terminal.
