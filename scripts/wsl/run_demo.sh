#!/usr/bin/env bash
# run_demo.sh — launch a demo on WSL2 with the environment created by setup_wsl.sh.
#
#   scripts/wsl/run_demo.sh vision               # vision shadow + 3D viewer (http://127.0.0.1:8301/index.html)
#   scripts/wsl/run_demo.sh vision-gui           # vision shadow with SUMO-GUI only (+ ffplay if installed)
#   scripts/wsl/run_demo.sh messages [CSV]       # V2I message shadow, 2 SUMO-GUI + viewer (http://127.0.0.1:8300/index.html)
#   scripts/wsl/run_demo.sh messages-smoke       # same, headless, 30 s (quick self-test)
#   scripts/wsl/run_demo.sh readout              # headless causal readout (writes verify_out/imagenes_finales/datos)
#
# Env: GRACE (seconds to open the browser, default 20), BASE (default ~/ds).
# The windows (SUMO-GUI) show up through WSLg (DISPLAY=:0); the 3D viewer is a normal web page,
# open it in the Windows browser (WSL forwards localhost).
set -euo pipefail

REPO="$(cd "$(dirname "$0")/../.." && pwd)"
BASE="${BASE:-$HOME/ds}"
VENV="$BASE/venv"
[ -x "$VENV/bin/python" ] || { echo "Run scripts/wsl/setup_wsl.sh first"; exit 1; }

export SUMO_HOME="$("$VENV/bin/python" -c 'import sumo; print(sumo.SUMO_HOME)')"
export PATH="$VENV/bin:$PATH"
export PYTHONPATH="$SUMO_HOME/tools:$REPO${PYTHONPATH:+:$PYTHONPATH}"
export DISPLAY="${DISPLAY:-:0}"
export GRACE="${GRACE:-20}"
cd "$REPO"

case "${1:-vision}" in
  vision)        exec python scripts/run_harness_demo_causal.py --3d --video tramo ;;
  vision-gui)    exec python scripts/run_harness_demo_causal.py ;;
  readout)       exec python scripts/run_harness_demo_causal.py --readout ;;
  messages)      cd reconstruccion_causal
                 CSV="${2:-inputs/cert/mc_2Hz_P00.csv}" PERIOD=0.5 OUT="runs/demo_$(date +%H%M%S)" \
                   exec ./scripts/vista_dos_ventanas_3d.sh ;;
  messages-smoke) cd reconstruccion_causal
                 CSV="inputs/cert/mc_2Hz_P00.csv" PERIOD=0.5 OUT=runs/smoke MODE=headless SMOKE_TMAX=30 GRACE=3 \
                   exec ./scripts/vista_dos_ventanas_3d.sh ;;
  *) sed -n '2,10p' "$0"; exit 2 ;;
esac
