#!/usr/bin/env bash
# vista_dos_ventanas_3d.sh — MODE 5: a REPLICA of vista_dos_ventanas.sh (which stays
# INTACT) + 3D WEB SHADOW. A SINGLE SIMULATION, THREE VIEWS: the two
# SUMO-GUI windows (reality :8892 | shadow :8893) and the 3D consume the SAME
# causal_player — zero second run, impossible for them to diverge.
#
# DELIBERATE differences from mode 2:
#   - 1x CADENCE: the ONLY authority over the pace is the realtime pacing (player
#     --ws-realtime + driver DRIVER_REALTIME, absolute wall/t_sim anchor). DELAY
#     is the SUMO-GUI RENDER THROTTLE, NOT a speed control: with the
#     pacing active its effect on the cadence is masked (measured: --delay
#     0/200 -> 1.00x the same). It is left at 50 (a mild harmless auto-throttle).
#     LATENT FOOTGUN: raising DELAY above the crossover with 1x (~1000 in light
#     render, less with two windows) makes it the bottleneck and DRAGS the setup
#     below 1x — the pacing only brakes, never accelerates. Do NOT raise it.
#   - ANCHOR HANDOFF (desync fix): the player writes $OUT/rt_anchor.txt at its
#     first emit; the driver reads it (DRIVER_RT_ANCHOR) and sets R(w)=S(w), so
#     that the shadow follows reality with an EXACT lag delay_s (1.0s), not the
#     ~0.48s compressed by sim_first + jitter. It is the replica latency made
#     visible (Section 7.2 of the paper), not cosmetic.
#   - player with --ws-log/--ws-port/--ws-realtime/--ws-open-grace and
#     --occupancy-out (certified passive layer; without it the HUD would go without a table).
#   - static webserver (ws_emitter.webserver, a SEPARATE process without TraCI) + URL
#     printed BEFORE the grace: there are $GRACE s to open the browser.
#   - the REALITY driver starts after sleep $GRACE: the grace freezes the
#     shadow (its SUMO waits for the player) and without this delay reality would
#     get ahead by exactly $GRACE s.
#   - trap: on exit NOTHING is left hanging — not the webserver, the player, the SUMOs, nor the driver.
#
# Usage: CSV=inputs/cert/mc_2Hz_P00.csv PERIOD=0.5 OUT=runs/x ./scripts/vista_dos_ventanas_3d.sh
#        MODE=headless SMOKE_TMAX=30 ... (smoke without GUI, for passivity tests)
set -euo pipefail
cd "$(dirname "$0")/.."

PORT_REAL=${PORT_REAL:-8892}
PORT_SOMBRA=${PORT_SOMBRA:-8893}
CSV=${CSV:-inputs/cert/mc_2Hz_P00.csv}
NET=inputs/intersection_rusia.net.xml
PERIOD=${PERIOD:-0.5}
OUT=${OUT:-runs/vista3d_$(date +%Y%m%d-%H%M)}
# ABSOLUTE path of the anchor: the driver cds to another dir, a relative $OUT would
# not resolve (anchor handoff). The player writes rt_anchor.txt under its --out
# with cwd here (reconstruccion_causal), so $(pwd)/$OUT is correct.
case "$OUT" in /*) OUT_ABS="$OUT" ;; *) OUT_ABS="$(pwd)/$OUT" ;; esac
DELAY=${DELAY:-50}                 # RENDER THROTTLE, not a speed control (see header)
GRACE=${GRACE:-20}                 # seconds to open the browser
MODE=${MODE:-gui}                  # gui | headless (smoke)
SMOKE_TMAX=${SMOKE_TMAX:-}
DRIVER_DIR=../reconstruccion_veins
REPO_ROOT=$(cd .. && pwd)

# 3D ports from config (web3d.yaml = single source; override by env)
SITE=${SITE:-rusia}
read -r CFG_HTTP_HOST CFG_HTTP_PORT CFG_WS_PORT < <(python3 -c "
import yaml
c = yaml.safe_load(open('$REPO_ROOT/config/sites/$SITE/web3d.yaml'))
print(c['http_host'], c['http_port'], c['ws_port'])")
HTTP_PORT=${HTTP_PORT:-$CFG_HTTP_PORT}
WS_PORT=${WS_PORT:-$CFG_WS_PORT}
HTTP_HOST=${HTTP_HOST:-$CFG_HTTP_HOST}

BIN=sumo-gui
GUI_REAL="--delay $DELAY --start --window-size 950,950 --window-pos 0,0"
GUI_SOMBRA="--delay $DELAY --start --window-size 950,950 --window-pos 960,0"
[ "$MODE" = headless ] && { BIN=sumo; GUI_REAL=""; GUI_SOMBRA=""; }

COMUN="--step-length 0.05 --time-to-teleport -1 --collision.action none \
       --ignore-route-errors --no-step-log true --end 100000"

PIDS=()
cleanup() {
  for p in "${PIDS[@]:-}"; do kill "$p" 2>/dev/null || true; done
}
# SESSION_DONE=1 once the player has finished and its outputs are written. From then on
# the only thing left is the 3D viewer / SUMO-GUI windows staying open for inspection, so
# closing them (Ctrl+C, SIGTERM) is the normal way to end the session and exits 0 instead of
# a signal status (130/143). An interruption BEFORE the run completes keeps the signal status.
SESSION_DONE=0
on_signal() {
  local sig_rc=$1
  cleanup
  [ "$SESSION_DONE" = 1 ] && exit 0
  exit "$sig_rc"
}
trap cleanup EXIT
trap 'on_signal 130' INT
trap 'on_signal 143' TERM

echo "[vista3d] REALIDAD :$PORT_REAL | SOMBRA :$PORT_SOMBRA | 3D ws:$WS_PORT http:$HTTP_PORT"
echo "[vista3d] csv=$CSV | delay=${DELAY}ms (1x=50) | gracia=${GRACE}s | out=$OUT"

$BIN -n "$NET" $COMUN --remote-port "$PORT_REAL"   $GUI_REAL &
PIDS+=($!); SUMO_REAL_PID=$!
$BIN -n "$NET" $COMUN --remote-port "$PORT_SOMBRA" $GUI_SOMBRA &
PIDS+=($!); SUMO_SOMBRA_PID=$!
sleep 2

# static webserver (separate process, without TraCI)
( cd "$REPO_ROOT" && python3 -m ws_emitter.webserver --site "$SITE" \
    --http-port "$HTTP_PORT" --ws-port "$WS_PORT" ) &
PIDS+=($!); WEB_PID=$!
sleep 1

echo
echo "=============================================================="
echo ">>> SOMBRA 3D:  http://$HTTP_HOST:$HTTP_PORT/index.html"
echo ">>> tienes ${GRACE}s (gracia) para abrirla antes de que arranque"
echo "=============================================================="
echo

mkdir -p "$OUT"
rm -f "$OUT/rt_anchor.txt"          # handoff: so the driver does not read an old anchor
python3 scripts/causal_player.py --csv "$CSV" --net "$NET" --out "$OUT" \
    --delay-s 1.0 --period-s "$PERIOD" --interp linear --guard-mode assert \
    --traci-port "$PORT_SOMBRA" --client-order 1 --server-step 0.05 \
    --shadow-color 255,20,60,220 --staging-mode park \
    --occupancy-out "$OUT/occ.csv" \
    --ws-log "$OUT/ws_log.jsonl" --ws-port "$WS_PORT" \
    --ws-realtime --ws-open-grace "$GRACE" &
PIDS+=($!); CAUSAL_PID=$!

# the grace freezes the SHADOW (its SUMO waits for the player); the REALITY
# driver waits the same so as not to get ahead by exactly $GRACE seconds
sleep "$GRACE"

DRIVER_ARGS=""
[ -n "$SMOKE_TMAX" ] && DRIVER_ARGS="--t-max $SMOKE_TMAX"
# DRIVER_REALTIME=1: REALITY at 1x with the same anchor pattern as the player
# (synchronization fix: without this, reality ran at 24x — the GUI --delay
# is a render floor, it does not hold back the TraCI client). Only mode 5 exports it.
# DRIVER_RT_ANCHOR: anchor handoff — the driver reads the player's anchor and sets
# R(w)=S(w) so that the visible lag is exactly delay_s (1.0s), not ~0.48s.
( cd "$DRIVER_DIR" && TRACI_PORT=$PORT_REAL DRIVER_REALTIME=1 \
    DRIVER_RT_ANCHOR="$OUT_ABS/rt_anchor.txt" python3 driver_replay.py $DRIVER_ARGS ) &
PIDS+=($!); DRIVER_PID=$!

if [ -n "$SMOKE_TMAX" ]; then
    wait "$DRIVER_PID" || true
    sleep 2
    kill "$SUMO_REAL_PID" "$SUMO_SOMBRA_PID" 2>/dev/null || true
    wait "$CAUSAL_PID" || true
    echo "[vista3d] smoke terminado; salidas en $OUT"
else
    wait "$DRIVER_PID" || true
    wait "$CAUSAL_PID" || true
    SESSION_DONE=1
    # The SUMO-GUI windows stay open on purpose (inspect the final state, 3D viewer still
    # served); close them or press Ctrl+C to end. Either way the launcher exits 0.
    wait "$SUMO_REAL_PID" "$SUMO_SOMBRA_PID" 2>/dev/null || true
    echo "[vista3d] sesión terminada; salidas en $OUT (ws_log + occ + summary)"
fi
exit 0
