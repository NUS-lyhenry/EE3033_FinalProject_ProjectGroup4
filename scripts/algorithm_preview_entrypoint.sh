#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
colcon --log-base /tmp/preview-colcon-logs build --symlink-install \
    --base-paths /work/src/ee3033_project --packages-select ee3033_project \
    --build-base /tmp/preview-build --install-base /tmp/preview-install
source /tmp/preview-install/setup.bash
preview_assets=/tmp/ee3033_sim/assets
mkdir -p "$preview_assets"
if [[ ! -f "$preview_assets/yolo26n.pt" ]]; then
    curl -fL --retry 2 -o "$preview_assets/yolo26n.pt" \
        https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo26n.pt
fi
printf '%s  %s\n' 9b09cc8bf347f0fc8a5f7657480587f25db09b34bf33b0652110fb03a8ad4fef "$preview_assets/yolo26n.pt" | sha256sum --check
if [[ ! -f "$preview_assets/cup.png" ]]; then
    curl -fL --retry 2 -o "$preview_assets/coco128.zip" \
        https://github.com/ultralytics/assets/releases/download/v0.0.0/coco128.zip
    python3 - "$preview_assets" <<'PY'
from pathlib import Path
import sys
from zipfile import ZipFile
root = Path(sys.argv[1])
with ZipFile(root/'coco128.zip') as archive:
    archive.extractall(root)
PY
    python3 scripts/prepare_simulation_assets.py "$preview_assets"
fi
exec python3 scripts/run_algorithm_preview.py --model "$preview_assets/yolo26n.pt" \
    --texture "$preview_assets/cup.png" --public-port "${PREVIEW_PUBLIC_PORT:-8792}"
