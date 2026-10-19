#!/bin/bash
# End-to-end check of the Spark setup. Run ON THE SPARK HOST from any clone:
#
#   scripts/spark/verify.sh            # everything
#   scripts/spark/verify.sh gl         # one section: host | triton | bench | gl | stream | isolation
#
# Prints real numbers (Triton latency/throughput under concurrent load,
# batch sizes, GL renderer per display path, NVENC utilisation) so the PR
# can state what was measured instead of what was assumed. Paste the whole
# output into the PR.
set -uo pipefail

REPO="$(cd "$(dirname "$0")/../.." && pwd)"
TRITON_TAG="${TRITON_TAG:-$(grep -h '^TRITON_TAG=' "$REPO/infra/triton/.env" 2>/dev/null | cut -d= -f2)}"
TRITON_TAG="${TRITON_TAG:-25.09}"
SECTIONS="${*:-host triton bench gl stream isolation}"

hdr() { printf '\n\033[1m== %s ==\033[0m\n' "$*"; }
ok()  { printf '  [ok]   %s\n' "$*"; }
bad() { printf '  [FAIL] %s\n' "$*"; }

# First running dev container (any developer's) — used for in-container checks.
dev_container() { docker ps --format '{{.Names}}' | grep -m1 -E '^halo-[a-z0-9-]+-px4-sitl-1$'; }

# ---------------------------------------------------------------------------
if [[ " $SECTIONS " == *" host "* ]]; then
hdr "host: driver, docker engine, CDI"
nvidia-smi --query-gpu=name,driver_version --format=csv,noheader
docker version --format 'docker {{.Server.Version}} ({{.Server.Os}}/{{.Server.Arch}})' 2>/dev/null
if docker info 2>/dev/null | grep -q 'nvidia.com/gpu='; then ok "CDI devices registered: $(docker info 2>/dev/null | grep -o 'nvidia.com/gpu=[^ ]*' | tr '\n' ' ')"
else bad "no nvidia.com/gpu CDI devices in 'docker info' — section 1 of docs/spark/README.md"; fi
if docker info 2>/dev/null | grep -qi 'desktop'; then bad "docker engine looks like Docker Desktop"; else ok "native docker engine"; fi
docker network inspect halo-inference >/dev/null 2>&1 && ok "halo-inference network exists" || bad "halo-inference network missing"
hdr "host: image tags have arm64 manifests"
for img in "nvcr.io/nvidia/tritonserver:${TRITON_TAG}-py3" "nvcr.io/nvidia/tritonserver:${TRITON_TAG}-py3-sdk" "nvcr.io/nvidia/tensorrt:${TRITON_TAG}-py3"; do
    if docker manifest inspect "$img" 2>/dev/null | grep -q '"architecture": "arm64"'; then ok "$img (arm64)"; else bad "$img has no arm64 manifest — change TRITON_TAG"; fi
done
fi

# ---------------------------------------------------------------------------
if [[ " $SECTIONS " == *" triton "* ]]; then
hdr "triton: server + model"
if docker ps --format '{{.Names}}' | grep -qx triton; then
    ok "triton container running ($(docker inspect -f '{{.Config.Image}}' triton))"
    docker exec triton curl -fsS localhost:8000/v2/health/ready >/dev/null && ok "server ready" || bad "server not ready"
    docker exec triton curl -fsS localhost:8000/v2/models/yolo_person/ready >/dev/null && ok "yolo_person READY" || bad "yolo_person not ready: docker logs triton"
    docker exec triton curl -fsS localhost:8000/v2/models/yolo_person/config | python3 -c 'import json,sys; c=json.load(sys.stdin); print("  max_batch_size", c["max_batch_size"], "dynamic_batching", c.get("dynamic_batching"))'
    echo "  driver on host: $(nvidia-smi --query-gpu=driver_version --format=csv,noheader) — compare with the container's minimum:"
    docker exec triton bash -c 'echo "  Triton $(cat /opt/tritonserver/TRITON_VERSION 2>/dev/null)  CUDA $(nvcc --version 2>/dev/null | grep -o "release [0-9.]*" || echo "?")"'
else
    bad "triton container not running: cd infra/triton && docker compose up -d"
fi
fi

# ---------------------------------------------------------------------------
if [[ " $SECTIONS " == *" bench "* ]]; then
hdr "bench: TensorRT engine under concurrent clients (perf_analyzer, gRPC)"
echo "  concurrency 1 = one developer; 2 and 5 = that many containers sending frames at once."
echo "  Look at: p50/p99 latency, 'Inferences/Second', and avg batch size below."
docker run --rm --network halo-inference "nvcr.io/nvidia/tritonserver:${TRITON_TAG}-py3-sdk" \
    perf_analyzer -m yolo_person -u triton:8001 -i grpc \
        --concurrency-range 1:5:2 --measurement-interval 5000 --percentile 99 2>&1 \
    | grep -E 'Concurrency|Inferences/Second|p50|p99|Avg request|Request count|batch|Failed|error' || bad "perf_analyzer failed"
echo "  avg batch size since server start (nv_inference_count / nv_inference_exec_count):"
docker exec triton curl -fsS localhost:8002/metrics 2>/dev/null \
    | awk '/^nv_inference_count\{.*yolo_person/{c=$2} /^nv_inference_exec_count\{.*yolo_person/{e=$2} END{ if (e>0) printf "  %.2f frames per GPU pass (%d requests / %d batches)\n", c/e, c, e; else print "  no inferences yet" }'
fi

# ---------------------------------------------------------------------------
if [[ " $SECTIONS " == *" gl "* ]]; then
hdr "gl: which display path gives GPU OpenGL (Xvfb / Xvfb+VirtualGL-EGL)"
c="$(dev_container)"
if [ -z "$c" ]; then bad "no halo-<dev>-px4-sitl-1 container running — open one in VS Code first"; else
    echo "  in $c:"
    printf '  (a) plain Xvfb:          '; docker exec -e DISPLAY=:0 "$c" bash -c 'glxinfo -B 2>/dev/null | grep -E "OpenGL renderer" || echo "glxinfo failed"'
    printf '  (b) Xvfb + vglrun -d egl: '; docker exec -e DISPLAY=:0 "$c" bash -c 'vglrun -d egl glxinfo -B 2>/dev/null | grep -E "OpenGL renderer" || echo "vglrun failed"'
    printf '  EGL devices seen:         '; docker exec "$c" bash -c 'eglinfo -B 2>/dev/null | grep -c "EGL device" || echo "eglinfo unavailable"'
    echo "  → (b) must say NVIDIA for Gazebo to be GPU-rendered. If (b) is llvmpipe/unavailable, the Xorg (glx-desktop) path is the fallback — report this."
fi
fi

# ---------------------------------------------------------------------------
if [[ " $SECTIONS " == *" stream "* ]]; then
hdr "stream: Selkies processes + NVENC utilisation"
c="$(dev_container)"
if [ -z "$c" ]; then bad "no dev container running"; else
    for p in Xvfb openbox turnserver selkies-gstreamer; do
        docker exec "$c" pgrep -f "$p" >/dev/null && ok "$p running" || bad "$p not running — docker exec $c cat /tmp/desktop/*.log"
    done
    echo "  Open the Selkies tab in a browser with Gazebo running, then watch 'enc' (NVENC %) — it must be > 0:"
    echo "  (10 samples, 1 s apart; sm = CUDA, enc = hardware encoder)"
    timeout 12 nvidia-smi dmon -s u -c 10 2>/dev/null || nvidia-smi --query-gpu=utilization.gpu,utilization.encoder --format=csv -l 1 -c 10 2>/dev/null || echo "  nvidia-smi dmon unsupported on this GPU — check 'nvidia-smi --query-gpu=utilization.encoder'"
fi
fi

# ---------------------------------------------------------------------------
if [[ " $SECTIONS " == *" isolation "* ]]; then
hdr "isolation: per-developer containers"
mapfile -t devs < <(docker ps --format '{{.Names}}' | grep -E '^halo-[a-z0-9-]+-px4-sitl-1$')
echo "  running dev containers (${#devs[@]}): ${devs[*]:-none}"
[ "${#devs[@]}" -ge 2 ] && ok "≥2 concurrent developer containers" || echo "  (need ≥2 running for the concurrency check — open a second clone)"
for c in "${devs[@]}"; do
    echo "  $c:"
    docker inspect -f '    networks: {{range $k,$v := .NetworkSettings.Networks}}{{$k}} {{end}}  host-mode: {{.HostConfig.NetworkMode}}' "$c"
    docker exec "$c" bash -lc 'echo "    ros2 nodes: $(ros2 node list 2>/dev/null | tr "\n" " ")"'
    docker exec "$c" bash -lc 'echo "    env: ROS_AUTOMATIC_DISCOVERY_RANGE=$ROS_AUTOMATIC_DISCOVERY_RANGE HALO_TRITON_URL=$HALO_TRITON_URL"'
    docker exec "$c" bash -c 'getent hosts triton >/dev/null && echo "    reaches triton: yes" || echo "    reaches triton: NO"'
done
echo "  With fly+perceive running in two containers, compare each detector's 'processed N Hz' and 'skipped' lines (ros2 topic echo / node logs): they should be similar — that's the no-starvation check."
fi
echo