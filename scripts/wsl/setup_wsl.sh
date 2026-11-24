#!/usr/bin/env bash
# setup_wsl.sh — user-space environment (no sudo) to run the demos on WSL2 / Ubuntu.
#
# Run it from a clone that lives INSIDE the WSL filesystem (e.g. ~/ds/Digital_Shadows),
# not from /mnt/c: a Windows checkout with core.autocrlf=true turns every file into CRLF
# (breaks the .sh scripts) and is slower.
#
# What it installs under $BASE (default ~/ds), nothing system-wide:
#   tools/  : a tiny venv holding `uv`
#   venv/   : Python 3.10 (via uv) + the demo dependencies + SUMO 1.12.0 (pip wheel)
# Not installed: torch / ultralytics / boxmot (only needed to regenerate tracks with a GPU).
# Still needed from apt (needs sudo): ffmpeg (provides ffplay) — optional, only for the video window.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/../.." && pwd)"
BASE="${BASE:-$HOME/ds}"
SUMO_VERSION="${SUMO_VERSION:-1.12.0}"   # README: SUMO 1.12. SUMO 1.28 crashes (exit 245) in this pipeline.
mkdir -p "$BASE"

echo "== 1. uv =="
if [ ! -x "$BASE/tools/bin/uv" ]; then
  rm -rf "$BASE/tools"
  python3 -m venv --without-pip "$BASE/tools"          # python3-venv/ensurepip may be missing
  python3 -m pip --python "$BASE/tools/bin/python" install -q uv
fi
UV="$BASE/tools/bin/uv"

echo "== 2. Python 3.10 + venv =="
"$UV" python install 3.10
[ -x "$BASE/venv/bin/python" ] || "$UV" venv --python 3.10 "$BASE/venv"
PY="$BASE/venv/bin/python"

echo "== 3. dependencies (without torch/CUDA/YOLO) =="
grep -viE '^(torch|torchvision|triton|nvidia-|cuda-|ultralytics|boxmot|filterpy|lapx|gdown|hf-xet|huggingface_hub|trackers|yacs)' \
  "$REPO/requirements.txt" | sed 's/[[:space:]]*#.*$//' | grep -v '^[[:space:]]*$' > "$BASE/req_demo.txt"
"$UV" pip install --python "$PY" --no-deps -r "$BASE/req_demo.txt" defusedxml

echo "== 4. SUMO $SUMO_VERSION =="
"$UV" pip install --python "$PY" "eclipse-sumo==$SUMO_VERSION" "traci==$SUMO_VERSION" "sumolib==$SUMO_VERSION"

echo "== 5. check =="
"$BASE/venv/bin/sumo" --version | head -1
"$PY" -c "import sumo; print('SUMO_HOME =', sumo.SUMO_HOME)"
"$PY" -c "import numpy, pandas, scipy, cv2, yaml, websockets, traci, sumolib, supervision; print('imports OK')" 2>/dev/null
command -v ffplay >/dev/null || echo "NOTE: ffplay not found -> 'sudo apt install ffmpeg' for the video window (the shadow runs without it)."
echo "Done. Next: scripts/wsl/run_demo.sh vision|messages"
