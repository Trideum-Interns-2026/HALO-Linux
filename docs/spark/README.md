# Developing on the DGX Spark (via Brev)

The Spark is a **headless shared host**. Nobody opens VS Code on its own
monitor any more. Each teammate connects from their laptop with Brev +
VS Code Remote-SSH, opens *their own* clone, and "Reopen in Container"
with the Spark profile. Everything — build, sim, Gazebo rendering, YOLO —
runs on the Spark's GPU; the laptop only shows a browser tab and an editor.

```
laptop (WSL2)                        DGX Spark
───────────────                      ─────────────────────────────────────────
VS Code ──Remote-SSH (Brev)──►  ~/devs/<you>/halo-linux  (your clone, .env HALO_DEV=<you>)
   │                                   │  Reopen in Container (Spark profile)
   │                                   ▼
   │  port-forward 8080/3478   halo-<you>-px4-sitl-1   ──┐
browser ◄── Selkies (NVENC) ──  Xvfb + Gazebo (VirtualGL)│  bridge: halo-inference
                                yolo_person_detector ────┼──► triton  (one TensorRT
                                                         │   engine, dynamic batching)
                                halo-<other>-px4-sitl-1 ─┘
```

Sections 1–3 are **one-time host setup** (whoever administers the Spark).
Sections 4–6 are **per developer**.

---

## 1. Host: native Docker + NVIDIA Container Toolkit

Docker Desktop's Linux engine runs in its own VM and never sees the host's
CDI GPU spec — that was the `nvidia.com/gpu` CDI failure. Replace it with
the native engine.

```bash
# remove Docker Desktop if present (it conflicts with docker-ce's socket)
sudo apt-get remove -y docker-desktop || true

# docker-ce (official repo) + nvidia-container-toolkit, then enable CDI
#   https://docs.docker.com/engine/install/ubuntu/
#   https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html
sudo nvidia-ctk runtime configure --runtime=docker
sudo nvidia-ctk cdi generate --output=/etc/cdi/nvidia.yaml
sudo systemctl restart docker

# this line is the actual fix for the earlier CDI error — it must list the GPU
docker info | grep -i -A3 cdi          # expect:  nvidia.com/gpu=all  /  nvidia.com/gpu=0
docker run --rm --device nvidia.com/gpu=all ubuntu nvidia-smi
```

## 2. Host: register with Brev

```bash
curl -fsSL https://brev.nvidia.com/install | bash
brev login
brev register "halo-dgx-spark"
brev enable-ssh
brev grant-ssh <teammate-brev-email>     # once per teammate
```

From here on treat the Spark as headless: no local desktop session, no
VS Code on its monitor.

## 3. Host: bring up the shared Triton inference server (once)

One Triton container serves one TensorRT YOLOv8n engine to every
developer's container. Build the engine **on the Spark** (engines are not
portable across GPU/driver/TensorRT versions), then start the server.

```bash
cd ~/devs/<you>/halo-linux/infra/triton       # any clone works
cp .env.example .env                            # TRITON_TAG — see note
./build_engine.sh                               # ~2-5 min: .pt → ONNX → model.plan
docker compose up -d
docker compose logs -f                          # wait for:  yolo_person | 1 | READY
```

`build_engine.sh` needs at least one `halo-base:<dev>` image to exist, so
if this is a brand new Spark do section 4 first, then come back.

**TRITON_TAG**: must have an arm64 (SBSA) image and a minimum driver no
newer than the Spark's. `scripts/spark/verify.sh` prints both; if it
complains, bump/lower the tag in `infra/triton/.env` **and** pass the same
`TRITON_TAG` in your repo `.env` so the client wheel comes from the same
release.

Restarting or upgrading Triton only interrupts detection: each developer's
`yolo_person_detector` skips frames while it's down and resumes by itself.

---

## 4. Developer: your clone on the Spark

We all log in as the same Linux user, so **your clone's `.env` is what
keeps your containers separate from everyone else's.**

```bash
# from your laptop, in WSL2
brev open halo-dgx-spark          # plain SSH shell on the Spark

# on the Spark
mkdir -p ~/devs/<you> && cd ~/devs/<you>
git clone git@github.com:Trideum-Interns-2026/halo-linux.git
cd halo-linux
echo "HALO_DEV=<you>" > .env       # lowercase letters/digits/- ; becomes the docker project name
git config user.name  "<Your Name>"       # repo-local: shared account, shared global gitconfig!
git config user.email "<you@example.com>"
```

Never work in someone else's directory under `~/devs/`.

## 5. Developer: VS Code from your laptop

Brev documents the Windows path as **WSL2 → `brev open <name> code`**.
One-time fix: VS Code launched from WSL2 would otherwise use the *Windows*
`ssh.exe`, which can't run Brev's tunnel helper. Follow Brev's Windows/WSL
troubleshooting guide to point `remote.SSH.path` at a small wrapper script
that calls WSL's `ssh`.

```bash
# laptop, WSL2
brev open halo-dgx-spark code
```

In the VS Code window that opens on the Spark: **File → Open Folder →
`~/devs/<you>/halo-linux`**, then **Dev Containers: Reopen in Container**
and pick **"PX4 SITL Dev Container (NVIDIA Spark)"**.

First open builds two images (`halo-base:<you>` from the unchanged root
Dockerfile, then `halo-spark:<you>` on top) — 10–20 minutes the first time
on the Spark, seconds afterwards thanks to the shared layer cache. If it
stops with *"HALO_DEV is not set"*, see section 4.

## 6. Developer: using it

- **Gazebo / the desktop**: VS Code's *Ports* tab shows **Selkies desktop**
  (8080) — click the globe. Log in with user = your `HALO_DEV`, password =
  `SELKIES_BASIC_AUTH_PASSWORD` from your `.env` (auto-generated on first
  open). Port 3478 is forwarded alongside it; leave it — that's the TURN
  relay that lets the WebRTC video follow your SSH tunnel.
- **`fly`, `perceive`, `track`, `spawn_person`** work as before. `fly` and
  `perceive` are wrapped in `vglrun -d egl` so Gazebo's OpenGL hits the
  NVIDIA GPU rather than Xvfb's software renderer. If you launch `gz sim`
  by hand, prefix it with `vglrun -d egl`.
- **YOLO** goes to the shared Triton automatically (`HALO_TRITON_URL` is set
  in the container). To run the old in-process model instead, e.g. to
  compare: `ros2 launch perception yolo_person_detection.launch.py` with
  `backend:=local` on the node, or `unset HALO_TRITON_URL` first. The
  detector's 5-second stats line shows `skipped N (inference unavailable)`
  if Triton is down.
- **Each container is network-isolated** (its own bridge; ROS 2 and Gazebo
  pinned to loopback inside it), so you'll never see a teammate's nodes in
  `ros2 node list`, and nothing you run can collide on a port with theirs.
- Your container survives VS Code disconnecting (`shutdownAction: none`).
  To free GPU memory when you're done for the day:
  `docker compose -p halo-<you> down`.

## Known limitation

With bridge networking, a QGroundControl on your laptop no longer receives
MAVLink from the sim (it used to, via `network_mode: host` + WSLg). Not
addressed in this change; the likely fix is running QGC inside the Selkies
desktop.

## Troubleshooting

| Symptom | Check |
|---|---|
| `nvidia.com/gpu` device not found | `docker info \| grep -i cdi`, section 1 |
| Selkies page loads but stays black / "connecting" | Port 3478 forwarded? `cat /tmp/desktop/coturn.log`, `/tmp/desktop/selkies.log` |
| Gazebo slow, `glxinfo -B` says llvmpipe | Run under `vglrun -d egl`; `vglrun -d egl glxinfo -B` should say NVIDIA |
| Detector logs `skipped N (inference unavailable)` | `docker compose -f infra/triton/docker-compose.yml ps`; is the dev container on `halo-inference`? |
| Someone else's drone in your sim | Your `.env` has the same `HALO_DEV` as theirs |

Run `scripts/spark/verify.sh` on the Spark host for an end-to-end check of
all of the above.
