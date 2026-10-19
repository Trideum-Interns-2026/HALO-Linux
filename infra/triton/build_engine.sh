#!/bin/bash
# Builds the TensorRT engine Triton serves, ON THE SPARK. Run from any
# directory; output lands in infra/triton/model_repository/yolo_person/1/.
#
# Two steps, each in a container so nothing has to be installed on the host:
#   1. YOLOv8n .pt → ONNX, using the Ultralytics already baked into the
#      project's dev image (the same /opt/yolo/yolov8n.pt weights every
#      container uses today).
#   2. ONNX → TensorRT plan with trtexec from NVIDIA's TensorRT container of
#      the SAME release as the Triton server, so the plan is built by exactly
#      the TensorRT version that will load it.
#
# Why not `YOLO(...).export(format='engine')`? Ultralytics writes its own
# metadata header in front of the serialized engine, which Triton's
# tensorrt backend can't deserialize, and it would build with whatever
# TensorRT pip resolved rather than Triton's. This still builds on the Spark
# itself — a .plan from any other GPU/driver/TRT combination is not portable.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
TRITON_TAG="${TRITON_TAG:-$(grep -h '^TRITON_TAG=' "$HERE/.env" 2>/dev/null | cut -d= -f2)}"
TRITON_TAG="${TRITON_TAG:-25.09}"
# Any developer's base image works — it only needs Ultralytics + the weights.
BASE_IMAGE="${BASE_IMAGE:-$(docker images --format '{{.Repository}}:{{.Tag}}' | grep -m1 '^halo-base:' || true)}"
OUT="$HERE/model_repository/yolo_person/1"
WORK="$HERE/.build"
IMGSZ=640

if [ -z "$BASE_IMAGE" ]; then
    echo "error: no halo-base:<dev> image found - open the repo in a Spark dev container once first (or set BASE_IMAGE)" >&2
    exit 1
fi

mkdir -p "$OUT" "$WORK"

echo "==> [1/2] yolov8n.pt -> ONNX  (in $BASE_IMAGE)"
docker run --rm -v "$WORK:/out" "$BASE_IMAGE" python3 - <<EOF
from ultralytics import YOLO
import shutil
# dynamic=True: batch dimension is free so Triton can batch; H/W fixed at
# $IMGSZ via the trtexec shapes below.
path = YOLO('/opt/yolo/yolov8n.pt').export(format='onnx', imgsz=$IMGSZ, dynamic=True, simplify=True, opset=17)
shutil.copy(path, '/out/yolov8n.onnx')
print('wrote', path)
EOF

echo "==> [2/2] ONNX -> TensorRT plan  (nvcr.io/nvidia/tensorrt:${TRITON_TAG}-py3, fp16)"
docker run --rm --device nvidia.com/gpu=all \
    -v "$WORK:/work" -v "$OUT:/out" \
    "nvcr.io/nvidia/tensorrt:${TRITON_TAG}-py3" \
    trtexec --onnx=/work/yolov8n.onnx --saveEngine=/out/model.plan --fp16 \
        --minShapes=images:1x3x${IMGSZ}x${IMGSZ} \
        --optShapes=images:4x3x${IMGSZ}x${IMGSZ} \
        --maxShapes=images:8x3x${IMGSZ}x${IMGSZ} \
        --skipInference

echo
echo "engine written to $OUT/model.plan"
echo "next: (cd $HERE && docker compose up -d && docker compose logs -f)"
