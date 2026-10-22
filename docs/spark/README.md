# Developing on the DGX Spark (via SSH)

The Spark is a **headless shared host**. Nobody opens VS Code on its own
monitor any more. Each teammate connects from their laptop with plain SSH
(VS Code Remote-SSH), opens *their own* clone, and "Reopen in Container"
with the Spark profile. Everything — build, sim, Gazebo rendering, YOLO —
runs on the Spark's GPU; the laptop only shows a browser tab and an editor.

```
laptop (WSL2)                        DGX Spark
───────────────                      ─────────────────────────────────────────
VS Code ──Remote-SSH (ssh)───►  ~/devs/<you>/halo-linux  (your clone, .env HALO_DEV=<you>)
   │                                   │  Reopen in Container (Spark profile)
   │                                   ▼
   │  port-forward 8080, 5760  halo-<you>-px4-sitl-1   ──┐
browser ◄── Selkies (NVENC) ──  Xvfb + Gazebo (VirtualGL)│  bridge: halo-inference
QGC ◄───── MAVLink/TCP ───────  socat ◄─UDP─ PX4 SITL    │
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

## 2. Host: SSH access

The Spark runs a stock OpenSSH server on port 22. Hostname `spark-d19c`,
lab-network address `10.1.30.146` (check with `hostname -I` if it moves).
Everyone logs in as the shared `ascteinterns` user with their **own SSH
key** — no passwords over the wire, and no third-party tunnel service.

```bash
# on the Spark, once per teammate: append the public key they send you
echo "<contents of teammate's id_ed25519.pub>" >> ~/.ssh/authorized_keys
chmod 700 ~/.ssh && chmod 600 ~/.ssh/authorized_keys

# optional hardening once every key is in
sudo sed -i 's/^#\?PasswordAuthentication .*/PasswordAuthentication no/' /etc/ssh/sshd_config
sudo systemctl restart ssh
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
# on your laptop (WSL2, macOS, or Windows PowerShell), once
ssh-keygen -t ed25519 -C "<you>@halo"      # send ~/.ssh/id_ed25519.pub to the Spark admin
cat >> ~/.ssh/config <<'EOF'
Host spark
    HostName 10.1.30.146
    User ascteinterns
    IdentityFile ~/.ssh/id_ed25519
    ServerAliveInterval 30
EOF
ssh spark                                   # plain SSH shell on the Spark

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

Install the **Remote - SSH** extension in VS Code. It reads the same
`~/.ssh/config` entry from section 4, so there is nothing Spark-specific
to configure. On Windows, put that entry in
`C:\Users\<you>\.ssh\config` (the built-in Windows `ssh.exe` works fine —
no WSL wrapper script needed), or if you prefer to drive VS Code from
WSL2, keep it in WSL's `~/.ssh/config` and launch `code` from there.

Connect with **Remote-SSH: Connect to Host… → `spark`** (or from a
terminal: `code --remote ssh-remote+spark ~/devs/<you>/halo-linux`).

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
  open). Page and video both ride that one forwarded port (Selkies 2.x
  streams over a WebSocket), so there is nothing else to forward.
- **QGroundControl runs on your laptop**, with your own gamepad, same as
  before — install it there if you haven't. What's different from WSL is
  the link: the sim's MAVLink never leaves your container, and the SSH
  tunnel only carries TCP, so QGC's usual UDP-14550 autoconnect finds
  nothing. Instead your container bridges the stream to TCP on **5760**,
  which VS Code forwards next to the Selkies port (*Ports* tab:
  "MAVLink → QGroundControl"). Once, in QGC: *Application Settings → Comm
  Links → Add* — type **TCP**, server address **localhost**, port **5760**,
  tick *Automatically Connect on Start*, then *Connect*. After that it's
  step 1 of the README's "Running it" as usual; the vehicle appears once
  `fly` is up. Everyone uses the same `localhost:5760` on their own laptop
  because each container has its own bridge and each tunnel is private —
  nothing is published on the Spark. One QGC per container at a time.
- **`desktop`** re-runs the whole bring-up (Xvfb, Selkies, MAVLink bridge)
  in place if something died; it's idempotent.
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

## Troubleshooting

| Symptom | Check |
|---|---|
| `nvidia.com/gpu` device not found | `docker info \| grep -i cdi`, section 1 |
| Selkies page spins forever / never loads | Nothing is listening on 8080 in *your* container. `desktop` in a container terminal (or `docker exec halo-<you>-px4-sitl-1 bash /usr/local/bin/start-desktop.sh` from the host). A forward made from a plain Remote-SSH window hits the Spark host, where nothing is published — it must be the window attached to the container |
| Selkies page loads but stays black / "connecting" | `cat /tmp/desktop/selkies.log` — first line should say `encodes H264 ... on nvenc`; if not, `NVIDIA_DRIVER_CAPABILITIES`/CDI (section 1) |
| QGroundControl says *Disconnected* / no vehicle | Link must be **TCP** `localhost:5760`, not UDP (section 6). 5760 in the *Ports* tab? `fly` running? `cat /tmp/desktop/mavlink-bridge.log` in the container; `desktop` restarts the bridge |
| QGC connects but a second person's QGC can't | Each container serves one QGC at a time; they need their own container (own clone + `HALO_DEV`), not yours |
| Gazebo slow, `glxinfo -B` says llvmpipe | Run under `vglrun -d egl`; `vglrun -d egl glxinfo -B` should say NVIDIA |
| Detector logs `skipped N (inference unavailable)` | `docker compose -f infra/triton/docker-compose.yml ps`; is the dev container on `halo-inference`? |
| Someone else's drone in your sim | Your `.env` has the same `HALO_DEV` as theirs |
| `ssh spark` hangs or is refused | Same network as the Spark? `ping 10.1.30.146`; key appended to `~/.ssh/authorized_keys` on the Spark (section 2)? |
| Remote-SSH connects but Selkies tab never opens | Check the *Ports* tab: 8080 must be forwarded (VS Code does this automatically for the Spark profile) |

Run `scripts/spark/verify.sh` on the Spark host for an end-to-end check of
all of the above.
