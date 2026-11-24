"""
causal_player — TraCI player of the CAUSAL reconstructor (batch with guard).

Structural differences from the collaborator's base (sombra_player.py):

  - There is no batch preprocessing: there is no build_schedule nor precomputed
    timeline. Each decision happens at the tick where it is decidable, and all
    message access passes through guardia_causal.GuardedMessageStore (criterion:
    no message with timestamp > t_decision + buffer; hard assert).
  - CREATION: on the arrival of the vehicle's first message (not at
    first_tick - create_margin known in advance). The reveal is still
    at tick_of(t0) + delay_ticks, so the insertion slack
    (~delay) is preserved without using the future.
  - ROUTE: born minimal (first arrived normal edge) and extended
    incrementally with each new edge that arrives (setRoute requires the
    list to start at the current edge — official TraCI docs —, so the
    extension is deferred while the vehicle is in staging or on an internal
    edge, with a per-tick retry).
  - REMOVAL: causal timeout (period + buffer of silence). The only termination
    mechanism; it does not distinguish the cause of the silence (see causal_core).
  - FINAL TAIL: the slots after the last message are extrapolated UNCERTAIN until
    the timeout (in the base they did not exist: the batch knew the end of the trace).

Per-tick flow (same order command -> step -> read as the base):
  1. clock + deliveries of arrived messages (creates trackers, extends routes)
  2. removals by timeout
  3. pending creations (add + minimal route)
  4. pending route extensions (retry)
  5. moveToXY: staging | due slot | interpolated frame | HELD
  6. simulationStep
  7. safeguards to newly present ones
  8. post-step read and validation (same columns as the base)

Usage:
  python3 scripts/causal_player.py --csv inputs/mc_2Hz_P30_rep000.csv \
      --net inputs/intersection_rusia.net.xml --out runs/<id> \
      --delay-s 1.0 --period-s 0.5 --interp linear [--guard-mode assert|audit]
"""

import argparse
import csv
import json
import math
import os
import sys
from collections import Counter

import sumolib
import traci

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import causal_core as core
from guardia_causal import GuardedMessageStore, CausalityViolation


def load_rows(csv_path):
    with open(csv_path) as f:
        return list(csv.DictReader(f))


_LANE_END_CACHE = {}


END_CLAMP_TOL_M = 1.5   # SUMO freezes the vehicle ~1 m before the lane end (measured)


def beyond_lane_end(lane_id, x, y, xr, yr, tol=END_CLAMP_TOL_M):
    """True if the commanded point (x, y) is at/after the END of lane_id (within
    `tol` m before it, along its last segment), the lane is a dead end (the net has
    no continuation there) and the point SUMO actually read (xr, yr) lags behind
    it along that segment.

    The CSV keeps moving a vehicle in a straight line past the end of the exit
    approaches (-E3, -E5, -E6 in the Rusia net), but the net stops there:
    moveToXY cannot place a vehicle where there is no road, so SUMO leaves it
    frozen at the edge end and the commanded-vs-read gap grows linearly. That gap is
    not a projection error of the shadow, it is data outside the modeled network,
    so the validation reports those ticks apart (FUERA_DE_RED) instead of FAIL.
    Only dead-end lanes qualify: on a connected lane a large error stays a FAIL.
    """
    info = _LANE_END_CACHE.get(lane_id)
    if info is None:
        try:
            shape = traci.lane.getShape(lane_id)
            links = traci.lane.getLinks(lane_id)
        except traci.TraCIException:
            shape, links = None, None
        info = False
        if shape and len(shape) >= 2 and not links:
            (x0, y0), (x1, y1) = shape[-2], shape[-1]
            seg = math.hypot(x1 - x0, y1 - y0)
            if seg > 0:
                info = (x1, y1, (x1 - x0) / seg, (y1 - y0) / seg)
        _LANE_END_CACHE[lane_id] = info
    if not info:
        return False
    ex, ey, ux, uy = info
    cmd_past = (x - ex) * ux + (y - ey) * uy > -tol
    read_lags = (xr - x) * ux + (yr - y) * uy < 0.0
    return cmd_past and read_lags


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", required=True)
    ap.add_argument("--net", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--margin", type=float, default=25.0)
    ap.add_argument("--gui", action="store_true")
    ap.add_argument("--ok-th", type=float, default=0.1)
    ap.add_argument("--warn-th", type=float, default=0.5)
    ap.add_argument("--strict-net", action="store_true",
                    help="count ticks past the end of dead-end exit lanes as "
                         "WARNING/FAIL instead of FUERA_DE_RED")
    ap.add_argument("--delay-s", type=float, default=1.0,
                    help="delay de emisión (s); el buffer de decisión es el mismo valor")
    ap.add_argument("--period-s", type=float, required=True,
                    help="periodo nominal (s) de la frecuencia: 1.0 / 0.5 / 0.2")
    ap.add_argument("--interp", choices=["none", "linear"], default="linear")
    ap.add_argument("--reacq-snap-th", type=float, default=3.0)
    ap.add_argument("--lost-after-s", type=float, default=3.0,
                    help="ventana de extrapolación: silencio tras el cual el "
                         "vehículo se da por PERDIDO y se retira (piso: "
                         "periodo+buffer). Umbral con análisis de sensibilidad "
                         "pendiente para el paper")
    ap.add_argument("--guard-mode", choices=["assert", "audit"], default="assert",
                    help="assert: CausalityViolation aborta; audit: contabiliza y sigue")
    ap.add_argument("--leak-probe", choices=["", "retiro_batch"], default="",
                    help="SOLO control negativo: reintroduce una fuga a propósito; "
                         "la corrida DEBE fallar con CausalityViolation")
    # --- ATTACH mode (comparison view): client of an already-running
    # multi-client SUMO (alongside the reality driver, order=2). It changes NO
    # reconstruction decision: only the TraCI transport and sub-step
    # re-commands so the shadow does not jitter with step-length < TICK.
    ap.add_argument("--traci-port", type=int, default=0,
                    help="conectarse a un SUMO existente en este puerto "
                         "(multi-cliente) en vez de lanzar uno propio")
    ap.add_argument("--client-order", type=int, default=1,
                    help="orden de cliente TraCI en modo attach")
    ap.add_argument("--server-step", type=float, default=0.05,
                    help="step-length del SUMO externo en modo attach; se "
                         "re-comanda el último estado en los sub-steps")
    ap.add_argument("--shadow-color", default="",
                    help="R,G,B,A para TODAS las sombras (vista comparada); "
                         "vacío = paleta por tipo")
    ap.add_argument("--trace-cmds", default="",
                    help="DIAGNÓSTICO: archivo donde volcar cada moveToXY "
                         "(t, veh, x, y, ang) con flush por línea, para "
                         "identificar el comando que mata al servidor")
    # --- DENSE dump (passive OBSERVATION layer):
    # one row per tick (0.1 s) per shadow vehicle with commanded state. It only
    # READS the reconstructor's state and the lane/edge that SUMO mapped
    # (TraCI subscriptions, 1 round trip per tick). It modifies NO
    # decision: passivity verified by byte-identical validation_log with and
    # without the dump, and 12/12 re-certification after integrating it.
    ap.add_argument("--dense-out", default="",
                    help="ruta del CSV denso (.gz => gzip en caliente); "
                         "vacío = sin volcado")
    # STAGING before the reveal:
    #   offnet (default, certified): teleport outside the network every tick —
    #     pattern inherited from the base. In sumo-gui 1.12 it triggers a NaN
    #     assert in moveToXY (render/TraCI race); headless is stable.
    #   park (views/GUI): the ghost is PARKED where SUMO inserted it
    #     (a valid point of the network), 100 % transparent and with velocity 0,
    #     commanded ONLY once; it is revealed (color + control) at its
    #     application tick. Pure presentation: it does not touch any causal decision.
    ap.add_argument("--staging-mode", choices=["offnet", "park"], default="offnet")
    # --- approach OCCUPANCY (paper metric):
    # per-logical-tick series in its OWN file (not mixed with the dense
    # dump). Passive observation layer, same criterion as --dense-out:
    # it only READS state, passivity verifiable by byte-identical validation_log.
    # stopped is decided with VAR_SPEED (SUMO's post-step speed), NEVER
    # with the message's speed_csv: a frozen frame must count as stopped.
    ap.add_argument("--occupancy-out", default="",
                    help="CSV de ocupación por carril de aproximación; "
                         "vacío = sin cómputo")
    ap.add_argument("--occupancy-config",
                    default=os.path.join(
                        os.path.dirname(os.path.abspath(__file__)), "..", "..",
                        "config", "sites", "rusia", "occupancy.yaml"),
                    help="YAML del sitio: umbral de halting y ref al lane_map")
    ap.add_argument("--occupancy-probe-out", default="",
                    help="DIAGNÓSTICO (humo): CSV por tick y vehículo con "
                         "lane/VAR_SPEED/elegibilidad, para auditar HELD->"
                         "detenido, ticks de snap y simetría de speed")
    ap.add_argument("--sumo-fcd-out", default="",
                    help="DIAGNÓSTICO (humo, solo standalone): pasa "
                         "--fcd-output al SUMO propio para cotejar VAR_SPEED "
                         "vs speed del fcd en la MISMA corrida")
    ap.add_argument("--ws-view", action="store_true",
                    help="construye el DecidedTickView por tick lógico en el punto "
                         "de acople WS (solo LECTURA de estado decidido, "
                         "pasividad verificada byte a byte; lo consume el emisor)")
    # --- WS emitter (architecture B): canonical log in the loop + queue +
    # broadcast thread. RULE: no network without log (the log is the truth of layer 3).
    ap.add_argument("--ws-log", default="",
                    help="log CANÓNICO JSONL del emisor (hello/steps/end, SIN "
                         "t_emit). Activa el view y el emisor; la red es opcional")
    ap.add_argument("--ws-port", type=int, default=0,
                    help="puerto del servidor WebSocket (broadcast); exige --ws-log")
    ap.add_argument("--ws-queue-max", type=int, default=1024,
                    help="tamaño de la cola loop->hilo; llena = descarte CONTADO "
                         "(el loop jamás bloquea; el log no pierde nada)")
    ap.add_argument("--ws-stall-drain", type=float, default=0.0,
                    help="DIAGNÓSTICO (control adverso de pasividad, espíritu "
                         "--leak-probe): retardo artificial del drain del hilo "
                         "para saturar la cola y ejercitar el descarte por Full")
    ap.add_argument("--ws-site", default="rusia",
                    help="site_id declarado en el hello del emisor")
    ap.add_argument("--ws-open-grace", type=float, default=0.0,
                    help="segundos de espera POST-hello y PRE-loop para que los "
                         "espectadores conecten (la corrida entera dura segundos; "
                         "sin gracia nadie llega al primer step). Pared pura, "
                         "pre-simulación: la dinámica no cambia (byte-compare)")
    ap.add_argument("--ws-realtime", action="store_true",
                    help="pacing SERVER-SIDE en TIEMPO REAL (único modo: la "
                         "sombra reproduce a la velocidad que "
                         "ocurrió, sin factor libre). ANCLADO A t_sim: el step "
                         "con t_sim=t se emite en el instante de pared t relativo "
                         "al PRIMER step (targets absolutos: un retraso no se "
                         "acumula). Demo/--3d: SIEMPRE; certificación/medición: "
                         "sin el flag (crudo). Pared pura pre-emisión: la "
                         "dinámica no cambia (byte-compare)")
    ap.add_argument("--shot-dir", default="",
                    help="Método B (demo 3 paneles): directorio donde volcar screenshots "
                         "PASIVOS de la sombra por tick, etiquetados por SIM (sh_<ms>.png). "
                         "gui.screenshot no altera dinámica (como render_split). Exige --gui.")
    ap.add_argument("--gui-settings-file", default="",
                    help="vista SUMO-GUI (config/sites/rusia/sumo_demo_view.xml): esquema de "
                         "color + tamaño de ventana. NO fija el encuadre — es manual del "
                         "operador (ver --manual-frame), no un rectángulo de config.")
    ap.add_argument("--manual-frame", action="store_true",
                    help="Método B (demo, video anexo): tras abrir la sumo-gui, PAUSA hasta "
                         "Enter para que el operador encuadre la vista a mano (zoom/pan). El "
                         "loop de captura arranca al Enter y no vuelve a tocar la vista "
                         "(simulationStep no la mueve). Exige --shot-dir y correr en primer "
                         "plano (stdin = terminal del operador).")
    args = ap.parse_args()
    if args.delay_s <= 0:
        ap.error("--delay-s debe ser > 0: el buffer causal es el propio delay")
    if args.shot_dir and not args.gui:
        ap.error("--shot-dir exige --gui (gui.screenshot necesita sumo-gui)")

    # --- DecidedTickView and emitter: lazy import — certification runs
    # without WS flags do not load the package.
    EMIT = bool(args.ws_log)
    if args.ws_port and not EMIT:
        ap.error("--ws-port exige --ws-log: no hay red sin log canónico")
    SHOT_DIR = args.shot_dir or ""          # screenshots of the shadow tagged by SIM
    MANUAL_FRAME = bool(args.manual_frame)  # manual framing before the capture loop
    if MANUAL_FRAME and not SHOT_DIR:
        ap.error("--manual-frame exige --shot-dir (no hay captura que encuadrar)")
    if SHOT_DIR:
        os.makedirs(SHOT_DIR, exist_ok=True)
    VIEW = bool(args.ws_view) or EMIT
    if VIEW:
        sys.path.insert(0, os.path.abspath(os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "..", "..")))
        from ws_emitter.view import DecidedTickView
    if EMIT:
        from ws_emitter.emitter import MessagesEmitter

    os.makedirs(os.path.join(args.out, "results"), exist_ok=True)
    os.makedirs(os.path.join(args.out, "logs"), exist_ok=True)

    # withInternal: the internal lanes are needed to derive the insertion edge
    # of vehicles that emit from inside the crossing. The bbox (and
    # therefore the coordinate transform) does not change — verified.
    net = sumolib.net.readNet(args.net, withInternal=True)
    (xmin, ymin), (xmax, ymax) = net.getBBoxXY()
    rows = load_rows(args.csv)
    store = GuardedMessageStore(rows, buffer_s=args.delay_s, mode=args.guard_mode)
    cfg = core.TrackerConfig(args.period_s, args.delay_s, args.delay_s,
                             xmin, ymax, args.margin, args.reacq_snap_th,
                             args.interp, args.lost_after_s)

    STAGING_X, STAGING_Y = xmin - 1000.0, ymin - 1000.0

    i_begin = core.tick_of(store.first_arrival_time())
    begin = max(0.0, (i_begin - 1) * core.TICK)
    HARD_CAP_TICKS = 200000   # fixed safety cap (not derived from the trace)

    attached = args.traci_port > 0
    n_sub = 1
    if attached:
        traci.init(port=args.traci_port, host="127.0.0.1")
        traci.setOrder(args.client_order)
        # sub-steps per tick: without re-command, SUMO would integrate the shadow
        # with its own dynamics between 0.1 s commands and it would appear to jitter
        n_sub = max(1, int(round(core.TICK / args.server_step)))
    else:
        sumo_bin = sumolib.checkBinary("sumo-gui" if args.gui else "sumo")
        cmd = [
            sumo_bin, "-n", args.net,
            "--step-length", str(core.TICK),
            "--begin", f"{begin:.2f}", "--end", "100000",
            "--time-to-teleport", "-1",
            "--collision.action", "warn",
            "--ignore-route-errors",
            "--no-step-log", "true",
        ]
        if args.gui and not SHOT_DIR:
            # AUTOMATIC START (--start): sumo-gui runs without waiting for the play button.
            # NOTE: with --start the DEFERRED gui.screenshot is not written
            # in this loop (0 shots) — the "running" mode does not redraw per TraCI step like
            # the paused mode. That is why the screenshot pass (SHOT_DIR) does NOT use --start: the
            # sim still advances via TraCI (simulationStep governs it, not the button) and the paused
            # mode redraws per step → the screenshot is written. The pass is AUTONOMOUS
            # (no manual play) because TraCI drives; the tag is the REAL sim (no offset).
            cmd += ["--start", "true", "--quit-on-end", "true"]
        if args.gui_settings_file:          # scheme + AA off + fixed window
            cmd += ["--gui-settings-file", args.gui_settings_file,
                    "--window-size", "1148,646"]
        if args.sumo_fcd_out:
            # smoke instrument (speed symmetry): fcd of the OWN standalone
            # SUMO; in attach the server is not ours and it does not apply
            cmd += ["--fcd-output", args.sumo_fcd_out]
        traci.start(cmd)
        # The framing is NOT set by config: it is manual by the operator (--manual-frame),
        # who frames by hand before the capture loop starts. See the pause
        # further below (right before the main loop).

    # shadow vTypes — copied from the base (geometry/color only; no effect on mobility)
    SHADOW_DIMS = {
        "shadow_car":   {"length": 5.0,  "width": 1.8,  "shape": "passenger", "color": (255, 230, 0, 255)},
        "shadow_bus":   {"length": 12.0, "width": 2.5,  "shape": "bus",       "color": (0, 150, 255, 255)},
        "shadow_truck": {"length": 7.5,  "width": 2.55, "shape": "truck",     "color": (255, 80, 0, 255)},
    }
    VTYPE_MAP = {"car": "shadow_car", "bus": "shadow_bus", "truck": "shadow_truck"}
    FALLBACK_TYPE = "shadow_car"
    for tid, d in SHADOW_DIMS.items():
        traci.vehicletype.copy("DEFAULT_VEHTYPE", tid)
        for setter, val in [("setMaxSpeed", 1000.0), ("setAccel", 1000.0),
                            ("setDecel", 1000.0), ("setEmergencyDecel", 1000.0),
                            ("setMinGap", 0.0), ("setImperfection", 0.0),
                            ("setLength", d["length"]), ("setWidth", d["width"]),
                            ("setShapeClass", d["shape"]), ("setVehicleClass", "ignoring")]:
            try:
                getattr(traci.vehicletype, setter)(tid, val)
            except traci.TraCIException:
                pass
        try:
            if args.shadow_color:
                traci.vehicletype.setColor(
                    tid, tuple(int(x) for x in args.shadow_color.split(",")))
            else:
                traci.vehicletype.setColor(tid, d["color"])
        except traci.TraCIException:
            pass

    trackers = {}          # veh -> VehicleTracker (exists from its first message)
    active = set()         # vehs created in SUMO and not removed
    pending_create = set() # trackers waiting to be created (e.g. no normal edge yet)
    k_next = {}            # veh -> next slot whose application has not happened yet
    last_applied = {}      # veh -> last commanded state (for HELD)
    last_k = {}            # veh -> last applied slot
    bridged_route = {}     # veh -> incremental bridged route (normal edges)
    route_dirty = set()    # vehs with new edges pending setRoute
    pending_safeguards = set()
    routes_added = set()

    vtype_usado = {t: 0 for t in SHADOW_DIMS}
    vtype_desconocidos = {}
    assigned_type = {}
    length_mismatches = []
    held_count = Counter()
    val_rows = []
    applied_this_tick = []   # (veh, state) commanded in the current tick
    collisions = 0
    teleports = 0
    creation_failures = []
    route_report = {"extensiones_ok": 0, "reintentos_diferidos": 0, "fallos": []}
    retiro_report = {}       # veh -> [episodes {last_arrival, retire_t, resume_t?}]
    generation = {}          # veh -> incarnation number in SUMO (re-acquisitions)
    reacquisitions = 0
    held_frames = 0          # frames frozen by the HELD-until-usable-anchor policy
    catchup_skipped = Counter()  # slots decided but not applied (absence/deferral)

    # crossing logger (copied from the base; measurement, does not affect mobility)
    crossing, last_inside, last_normal = {}, {}, {}
    try:
        J5_POLY = net.getNode("J5").getShape()
    except Exception:
        J5_POLY = None

    def in_junction(px, py):
        if not J5_POLY:
            return False
        c, j = False, len(J5_POLY) - 1
        for i2 in range(len(J5_POLY)):
            xi, yi = J5_POLY[i2]; xj, yj = J5_POLY[j]
            if ((yi > py) != (yj > py)) and (px < (xj - xi) * (py - yi) / (yj - yi + 1e-12) + xi):
                c = not c
            j = i2
        return c

    trace_f = open(args.trace_cmds, "w", buffering=1) if args.trace_cmds else None

    # --- dense dump: writer + subscriptions (only if --dense-out) ---
    DENSE = bool(args.dense_out)
    dense_f = dense_w = None
    dense_rows_n = 0
    if DENSE:
        import gzip
        os.makedirs(os.path.dirname(args.dense_out) or ".", exist_ok=True)
        dense_f = (gzip.open(args.dense_out, "wt", newline="")
                   if args.dense_out.endswith(".gz")
                   else open(args.dense_out, "w", newline=""))
        dense_w = csv.writer(dense_f)
        dense_w.writerow(["sim_time_s", "vehicle_id", "x", "y", "lane_sumo",
                          "edge_sumo", "lane_state", "speed", "headingDeg",
                          "provenance", "sim_time_csv", "generation"])
    VAR_ROAD, VAR_LANE = 0x50, 0x51   # traci.constants VAR_ROAD_ID / VAR_LANE_ID
    VAR_SPEED = 0x40                  # traci.constants VAR_SPEED
    VAR_POS = 0x42                    # traci.constants VAR_POSITION — verified in
                                      # environment: SUMO network plane [m,m],
                                      # the SAME as moveToXY/getPosition (this net has
                                      # no geo projection: projParameter="!")
    VAR_ANGLE = 0x43                  # traci.constants VAR_ANGLE — verified in
                                      # environment: post-step heading for the
                                      # view; the convention is converted in the emitter

    # --- occupancy: shared rule (occupancy_core) + lanes from the config
    # lane_map (NEVER from the net). The md5 of the lane_map actually read is
    # recorded in the summary: each run declares against which mapping it counted
    # (traceability without touching the existing pins).
    OCC = bool(args.occupancy_out)
    occ_f = occ_w = occ_probe_f = occ_probe_w = None
    occ_rows_n = 0
    if OCC:
        import hashlib
        import yaml
        import occupancy_core as occ_core
        occ_cfg_path = os.path.abspath(args.occupancy_config)
        with open(occ_cfg_path) as f:
            occ_cfg = yaml.safe_load(f)
        # lane_map_ref is relative to the root of implementacion/; the config
        # lives in <root>/config/sites/rusia/ -> root = three levels up
        occ_repo_root = os.path.abspath(os.path.join(
            os.path.dirname(occ_cfg_path), "..", "..", ".."))
        lane_map_path = os.path.join(occ_repo_root, occ_cfg["lane_map_ref"])
        with open(lane_map_path) as f:
            approach_lanes = occ_core.load_approach_lanes(yaml.safe_load(f))
        occ_halt = float(occ_cfg["halting_speed_mps"])
        with open(lane_map_path, "rb") as f:
            occ_lane_map_md5 = hashlib.md5(f.read()).hexdigest()
        os.makedirs(os.path.dirname(args.occupancy_out) or ".", exist_ok=True)
        occ_f = open(args.occupancy_out, "w", newline="")
        occ_w = csv.writer(occ_f)
        occ_w.writerow(["sim_time_s", "brazo", "carril", "ocupacion", "detenidos"])
        if args.occupancy_probe_out:
            os.makedirs(os.path.dirname(args.occupancy_probe_out) or ".",
                        exist_ok=True)
            occ_probe_f = open(args.occupancy_probe_out, "w", newline="")
            occ_probe_w = csv.writer(occ_probe_f)
            occ_probe_w.writerow(["sim_time_s", "veh", "lane_sumo",
                                  "var_speed", "estado"])

    def dense_provenance(st):
        seg = st.get("seg") or ""
        if st["provenance"] == "INTERPOLATED" and seg.endswith("(DR)"):
            return "DR"
        return st["provenance"]
    PARK = args.staging_mode == "park"
    pending_park = set()   # created but not yet parked (waiting to be present)
    parked = set()         # transparent parked ones, pending reveal

    def shadow_rgba(veh):
        if args.shadow_color:
            return tuple(int(x) for x in args.shadow_color.split(","))
        return SHADOW_DIMS[assigned_type[veh]]["color"]

    # NOTE: the VEINS source writes an invalid headingDeg
    # (TraCI INVALID_DOUBLE_VALUE sentinel, -1073741824) ROUNDED to 6
    # digits (-1.07374e+09) in the messages without heading (empty laneId): on
    # losing precision it stops being the legal sentinel and reaches SUMO as a
    # real garbage angle, which in sumo-gui 1.12 sometimes triggers the NaN
    # assert of moveToXY. Sanitizing AT THE COMMAND BOUNDARY: any
    # degenerate angle is sent as the EXACT sentinel, whose official semantics is
    # "the vehicle assumes the edge's natural angle". Pure presentation:
    # the reconstructed state (and its log) do not change.
    ANGLE_INVALID = -1073741824.0   # traci.constants.INVALID_DOUBLE_VALUE

    # ROOT CAUSE OF THE GUI CRASH: in SUMO 1.12, libsumo::Vehicle::moveToXY
    # declares `double lanePos;` UNinitialized; if the mapping does not find a
    # lane (position outside the network) with keepRoute=2 (mayLeaveNetwork),
    # `assert(!ISNAN(lanePos))` reads stack garbage -> aborts sometimes (non-
    # deterministic; the GUI stack differs from the headless one). The commands
    # outside the network are the offnet staging and the UNCERTAIN extrapolations
    # of the silence tail of vehicles that leave the map. In park mode every
    # command is CLAMPED to the network bbox (the ghost that leaves the map
    # stays drawn at the border until its removal). Pure presentation: the
    # reconstructed state and its log do not change.
    CLAMP_EPS = 0.1

    def moveToXY_traced(veh, x, y, ang, tag):
        if not (-360.0 <= ang <= 720.0):
            ang = ANGLE_INVALID
        if PARK:
            x = min(max(x, xmin + CLAMP_EPS), xmax - CLAMP_EPS)
            y = min(max(y, ymin + CLAMP_EPS), ymax - CLAMP_EPS)
        if trace_f:
            trace_f.write(f"{traci.simulation.getTime():.2f} {veh} {x:.3f} {y:.3f} {ang:.1f} {tag}\n")
        traci.vehicle.moveToXY(veh, "", -1, x, y, angle=ang, keepRoute=2)

    def apply_moveToXY(veh, st):
        moveToXY_traced(veh, st["sx"], st["sy"], st["angle"], "state")

    def resolve_exit_edge(edge_id):
        """NORMAL insertion edge for a vehicle that has (so far) only emitted
        from the crossing's internal lanes: it follows the outgoing connections
        of the internal lane in the STATIC NETWORK up to the first normal edge.
        Causal: it uses infrastructure, not the trace. The resolved edge may not
        be the real exit (the first connection is taken); it is only an insertion
        point — the position is imposed by moveToXY and the route is corrected when
        observed normal edges arrive. It starts from the message's edgeId
        (the internal laneId is ambiguous to parse in sumolib)."""
        try:
            cur = net.getEdge(edge_id).getLanes()[0]
            seen = set()
            while cur is not None and cur.getEdge().getFunction() == "internal":
                if cur.getID() in seen:
                    return None
                seen.add(cur.getID())
                outs = cur.getOutgoing()
                cur = outs[0].getToLane() if outs else None
            return cur.getEdge().getID() if cur is not None else None
        except Exception:
            return None

    def extend_bridged(veh, new_edge):
        """Extends the bridged route with findRoute (a query to the NETWORK, not the
        trace: causal). Same bridging as the base, but incremental."""
        br = bridged_route.setdefault(veh, [])
        if not br:
            br.append(new_edge)
            return
        if new_edge == br[-1]:
            return
        try:
            rr = traci.simulation.findRoute(br[-1], new_edge)
            seg = list(rr.edges) if rr and rr.edges else [new_edge]
        except traci.TraCIException:
            seg = [new_edge]
        for e in seg:
            if e != br[-1]:
                br.append(e)

    def try_set_route(veh):
        """setRoute requires the list to start at the vehicle's CURRENT edge
        (official TraCI docs). If it is in staging, on an internal edge or outside
        the known route, it is deferred and retried on the next tick."""
        br = bridged_route.get(veh) or []
        if not br:
            return True   # nothing to set yet
        try:
            cur = traci.vehicle.getRoadID(veh)
        except traci.TraCIException:
            return False
        if not cur or cur.startswith(":") or cur not in br:
            route_report["reintentos_diferidos"] += 1
            return False
        try:
            traci.vehicle.setRoute(veh, br[br.index(cur):])
            route_report["extensiones_ok"] += 1
            return True
        except traci.TraCIException as e:
            route_report["fallos"].append({"veh": veh, "err": str(e)})
            return False

    # --- WS emitter: hello BEFORE the first simulationStep (first
    # line of the log, verifiable). REAL md5 of the reproduced artifacts.
    ws_em = None
    if EMIT:
        import hashlib as _hl

        def _md5f(path):
            with open(path, "rb") as fh:
                return _hl.md5(fh.read()).hexdigest()

        hello_fields = {
            "shadow": "messages", "mode": "replay", "dt": core.TICK,
            "site": args.ws_site,
            "frame_geometry": None,   # messages profile: no video
                                      # (bbox_px does not exist in this shadow)
            "net_md5": _md5f(args.net),
            "artifact_md5": ({"mensajes_csv": _md5f(args.csv)}
                             | ({"lane_map": occ_lane_map_md5} if OCC else {})),
            "occ_cfg_md5": ({"occupancy_yaml": _md5f(occ_cfg_path)} if OCC else {}),
            "guard_mode": args.guard_mode,
            "zone_coverage": None, "layers": ["sumo"],
            "vclass_dims": {cls: {"length_m": SHADOW_DIMS[tid]["length"],
                                  "width_m": SHADOW_DIMS[tid]["width"]}
                            for cls, tid in VTYPE_MAP.items()},
            "occupancy_semantics": ({"kind": "sumo_lanes",
                                     "halting_speed_mps": occ_halt} if OCC else
                                    {"kind": "sumo_lanes",
                                     "note": "corrida sin --occupancy-out: "
                                             "per_lane vacío por step"}),
        }
        ws_em = MessagesEmitter(log_path=args.ws_log, hello_fields=hello_fields,
                                ws_port=args.ws_port,
                                queue_max=args.ws_queue_max,
                                stall_drain_s=args.ws_stall_drain)
        ws_em.emit_hello()
        if args.ws_port and args.ws_open_grace > 0:
            # connection grace: PRE-loop and PRE-first-step — a wall-clock pause
            # that does not touch any decision (passivity by byte-compare)
            import time as _time
            _time.sleep(args.ws_open_grace)

    # server-side REALTIME pacing (the only mode; anchored to t_sim, not to the
    # tick grid — in standalone they coincide, in attach/others it is not guaranteed).
    # The anchor (wall0, t_sim0) is fixed at the FIRST
    # emitted step; the targets are ABSOLUTE against that anchor (implicit
    # re-anchoring: a late step does not accumulate sleep debt).
    WS_REALTIME = bool(args.ws_realtime) and EMIT
    ws_pace_sleep = 0.0
    ws_rt_anchor = None      # (wall0, t_sim0) at the first step
    ws_rt_last = None        # (wall, t_sim) of the last emitted step (drift)
    if WS_REALTIME:
        import time as _ptime

    if MANUAL_FRAME:
        # the operator frames the sumo-gui by hand (zoom/pan) and
        # presses Enter. From then on the view stays fixed for the whole capture loop
        # (simulationStep does not move the view). It requires running in the foreground: stdin is
        # the operator's terminal. The pause is pure wall time (it touches no decision;
        # passivity = byte-identical validation_log, Gate 1).
        print("\n[MANUAL-FRAME] Encuadra la ventana SUMO-GUI (zoom/pan) y pulsa Enter "
              "para iniciar la captura...", flush=True)
        try:
            input()
        except EOFError:
            print("[MANUAL-FRAME] stdin sin terminal interactiva; arranco sin pausa.",
                  flush=True)

    guard_failure = None
    i = i_begin
    try:
        while True:
            t = round(i * core.TICK, 4)
            store.set_clock(t)
            applied_this_tick = []
            dense_this_tick = []

            # 1. deliveries: create trackers / update last_arrival / new edges.
            # If the vehicle was removed by timeout and its messages resume,
            # it is RE-ACQUIRED: the tracker persists (the t0 grid and arrived
            # history are causal knowledge) and only the SUMO instance is re-created.
            for msg in store.deliver_until(t):
                tr = trackers.get(msg.veh)
                if tr is None:
                    trackers[msg.veh] = tr = core.VehicleTracker(msg, cfg)
                    k_next[msg.veh] = 0
                    generation[msg.veh] = 0
                    pending_create.add(msg.veh)
                    for e in tr.norm_edges:
                        extend_bridged(msg.veh, e)
                    if args.leak_probe == "retiro_batch":
                        # NEGATIVE CONTROL: the base's batch logic knew the
                        # LAST message of the vehicle when creating it (removal at
                        # last_tick+1). We reproduce that access: it must FAIL.
                        view = store.slot_view(msg.veh, msg.t)
                        m_last = view.peek_absolute(-1, "leak_probe:retiro_batch")
                        _ = m_last.t  # the base used this timestamp for remove_at
                else:
                    was_retired = tr.retired
                    if tr.on_message(msg):
                        extend_bridged(msg.veh, tr.norm_edges[-1])
                        route_dirty.add(msg.veh)
                    if was_retired:
                        tr.retired = False
                        generation[msg.veh] += 1
                        reacquisitions += 1
                        if retiro_report.get(msg.veh):
                            retiro_report[msg.veh][-1]["resume_t"] = t
                        pending_create.add(msg.veh)
                        last_applied[msg.veh] = None   # staging until the next slot

            # 2. removals by causal timeout (the only termination mechanism).
            # The timeout does NOT distinguish the cause of the silence (loss, leaving
            # the map, shutdown): any silence > (period + buffer) removes. If the
            # messages resume later, step 1 re-acquires.
            for veh, tr in trackers.items():
                if tr.retired or not tr.timed_out(t):
                    continue
                tr.retired = True
                tr.retire_tick = i
                retiro_report.setdefault(veh, []).append(
                    {"last_arrival": round(tr.last_arrival, 4), "retire_t": t})
                pending_create.discard(veh)
                pending_park.discard(veh)
                parked.discard(veh)
                if veh in active:
                    try:
                        traci.vehicle.remove(veh)
                    except traci.TraCIException:
                        pass
                    active.discard(veh)

            # 3. pending creations. Insertion edge: the last arrived normal
            # one; if the vehicle only emitted from internal lanes,
            # it is derived from the static network (resolve_exit_edge).
            # DETERMINISTIC ORDER: every loop over a set whose
            # effects touch an output (logs, summary, trace, commands) iterates
            # in sorted order — the order of a set depends on the process hash seed
            # and made the validation_log not byte-reproducible.
            for veh in sorted(pending_create):
                tr = trackers[veh]
                if tr.norm_edges:
                    ins_edge = tr.norm_edges[-1]
                else:
                    ins_edge = resolve_exit_edge(tr.last_edge)
                    if ins_edge is None:
                        continue   # no resolvable edge yet: deferred
                rid = f"r_{veh}_g{generation[veh]}"
                try:
                    if rid not in routes_added:
                        traci.route.add(rid, [ins_edge])
                        routes_added.add(rid)
                    vt = tr.vtype
                    tid = VTYPE_MAP.get(vt, FALLBACK_TYPE)
                    if vt and vt not in VTYPE_MAP:
                        vtype_desconocidos[vt] = vtype_desconocidos.get(vt, 0) + 1
                    vtype_usado[tid] += 1
                    assigned_type[veh] = tid
                    traci.vehicle.add(veh, rid, typeID=tid, depart="now",
                                      departLane="free", departPos="free",
                                      departSpeed="0")
                    active.add(veh)
                    pending_safeguards.add(veh)
                    if PARK:
                        pending_park.add(veh)
                    if not bridged_route.get(veh):
                        extend_bridged(veh, ins_edge)   # seed = insertion edge
                    route_dirty.add(veh)
                    pending_create.discard(veh)
                except traci.TraCIException as e:
                    creation_failures.append({"veh": veh, "t": t, "err": str(e)})
                    pending_create.discard(veh)

            # 4. pending route extensions
            present = set(traci.vehicle.getIDList())
            for veh in sorted(route_dirty):
                if veh not in active:
                    route_dirty.discard(veh)
                    continue
                if veh in present and try_set_route(veh):
                    route_dirty.discard(veh)

            # 5. movement commands
            for veh in sorted(active):   # order of val_rows/dense/trace
                if veh not in present:
                    continue   # just added: inserted in the step below
                tr = trackers[veh]
                kn = k_next[veh]
                # catch-up: slots whose application tick passed while the
                # vehicle was absent (removal window) or its creation was
                # deferred: they are DECIDED (they keep the sequence and the snap) but
                # not applied.
                while tr.app_tick(kn) < i:
                    tr.decide_up_to(kn, store)
                    catchup_skipped[veh] += 1
                    last_k[veh] = kn
                    kn += 1
                    k_next[veh] = kn
                if tr.app_tick(kn) == i:
                    tr.decide_up_to(kn, store)
                    st = tr.states[kn]
                    apply_moveToXY(veh, st)
                    if veh in parked:
                        # reveal: restore color and give back speed
                        # control (the parking was pure presentation)
                        try:
                            traci.vehicle.setColor(veh, shadow_rgba(veh))
                            traci.vehicle.setSpeed(veh, -1)
                        except traci.TraCIException:
                            pass
                        parked.discard(veh)
                    last_applied[veh] = st
                    last_k[veh] = kn
                    k_next[veh] = kn + 1
                    applied_this_tick.append((veh, st))
                    if DENSE:
                        dense_this_tick.append((veh, st))
                elif last_applied.get(veh) is None:
                    # waiting for the next application tick (initial
                    # creation or re-acquisition after removal)
                    if PARK:
                        pass   # transparent parked where SUMO inserted it
                    else:
                        moveToXY_traced(veh, STAGING_X, STAGING_Y, 0.0, "staging")
                elif args.interp == "linear":
                    ka = last_k[veh]
                    tr.decide_up_to(ka + 1, store)   # closing anchor of the segment
                    st = tr.frame_between(ka, i)
                    if st.get("held_frame"):
                        held_frames += 1
                    # watched invariant: the shown state cannot
                    # embed messages newer than (frame instant) +
                    # buffer; with the HELD policy the frozen frame derives
                    # only from the past and this check is satisfied
                    s_frame = t - args.delay_s
                    store.check_state_use(st["max_msg_ts"], s_frame,
                                          f"interp_frame:{veh}@tick{i}")
                    apply_moveToXY(veh, st)
                    last_applied[veh] = st
                    applied_this_tick.append((veh, st))
                    if DENSE:
                        dense_this_tick.append((veh, st))
                else:
                    apply_moveToXY(veh, last_applied[veh])   # HELD
                    held_count[veh] += 1
                    if DENSE:
                        dense_this_tick.append((veh, last_applied[veh]))

            # 6. step
            traci.simulationStep(t)
            sim_after = traci.simulation.getTime()
            # PASSIVE screenshot of the shadow, tagged by SIM (t_s=sim_after)
            # → the compositor matches by sim. gui.screenshot only reads/renders (it does not touch
            # dynamics; passivity = byte-identical validation_log, gate 1). The tag is sim_after
            # (the REAL sim that the loop advanced), NOT a frame count since the window
            # start (no offset). The framing is set by the operator by hand before the loop
            # (--manual-frame) and is not touched during the capture: simulationStep does not move the
            # view, so all shots share the chosen framing.
            if SHOT_DIR:
                traci.gui.screenshot(
                    "View #0",
                    os.path.join(SHOT_DIR, f"sh_{int(round(sim_after * 1000)):09d}.png"),
                    width=1148, height=646)
            try:
                collisions += traci.simulation.getCollidingVehiclesNumber()
                teleports += traci.simulation.getStartingTeleportNumber()
            except traci.TraCIException:
                pass

            # 7. defensive safeguards to newly present ones (copied from the base)
            present = set(traci.vehicle.getIDList())
            for veh in sorted(pending_safeguards):
                if veh in present:
                    try:
                        traci.vehicle.setSpeedMode(veh, 0)
                        traci.vehicle.setLaneChangeMode(veh, 0)
                        exp_len = SHADOW_DIMS[assigned_type[veh]]["length"]
                        got_len = traci.vehicle.getLength(veh)
                        if abs(got_len - exp_len) > 0.01:
                            length_mismatches.append((veh, assigned_type[veh],
                                                      exp_len, got_len))
                        if DENSE or OCC or VIEW:
                            # VAR_SPEED: SUMO's speed after moveToXY derives
                            # from the displacement -> a frozen frame gives ~0 and
                            # counts as stopped: the metric measures what the
                            # shadow REPRESENTS, not what the message said
                            # VAR_POS/VAR_ANGLE: post-step pos_plane and heading of
                            # ALL revealed shadows for the WS emitter
                            # (v3 contract Section 5.1 and 3.3) — same step cache,
                            # no extra TraCI round trips
                            traci.vehicle.subscribe(
                                veh, [VAR_ROAD, VAR_LANE, VAR_SPEED, VAR_POS,
                                      VAR_ANGLE])
                        pending_safeguards.discard(veh)
                    except traci.TraCIException:
                        pass
            # transparent parking (park mode): ONLY once upon becoming present
            for veh in sorted(pending_park):
                if veh in present:
                    try:
                        traci.vehicle.setSpeed(veh, 0.0)
                        rgba = shadow_rgba(veh)
                        traci.vehicle.setColor(veh, (rgba[0], rgba[1], rgba[2], 0))
                        pending_park.discard(veh)
                        parked.add(veh)
                    except traci.TraCIException:
                        pass  # retry on the next tick

            # 8. post-step validation of what was commanded in this tick
            for veh, st in applied_this_tick:
                if veh not in present:
                    continue
                xr, yr = traci.vehicle.getPosition(veh)
                ar = traci.vehicle.getAngle(veh)
                err = math.hypot(xr - st["sx"], yr - st["sy"])
                ang_err = (ar - st["angle"] + 180) % 360 - 180
                status = ("OK" if err <= args.ok_th
                          else "WARNING" if err <= args.warn_th else "FAIL")
                # data past the end of a dead-end exit lane: not representable in
                # the net, reported apart (see beyond_lane_end). --strict-net
                # restores the raw classification.
                if (status != "OK" and not args.strict_net
                        and beyond_lane_end(st["lane_csv"], st["sx"], st["sy"],
                                            xr, yr)):
                    status = "FUERA_DE_RED"
                prov = st["provenance"]
                scsv = st["sim_time_csv"]
                val_rows.append({
                    "tick_i": i, "sim_time_target": t, "sim_time_after_step": sim_after,
                    "sim_time_csv": scsv, "quant_error_s": st["quant_error_s"],
                    "delay_s": args.delay_s,
                    "delay_efectivo_s": (round(t - scsv, 4) if scsv is not None else ""),
                    "alpha": st["alpha"],
                    "veh": veh, "x_cmd": st["sx"], "y_cmd": st["sy"],
                    "heading_cmd": st["angle"],
                    "x_read": xr, "y_read": yr, "angle_read": ar,
                    "position_error_m": err, "angle_error_deg": ang_err,
                    "projection_status": status, "provenance": prov,
                    "seg": st["seg"],
                    "speed_csv": st["speed_csv"], "accel_csv": st["accel_csv"],
                    "edge_csv": st["edge_csv"], "lane_csv": st["lane_csv"],
                })
                # crossing logger (copied from the base)
                lane = st["lane_csv"]
                is_norm = not lane.startswith(":")
                ins = in_junction(st["sx"], st["sy"])
                rec = crossing.setdefault(veh, {})
                prev_in = last_inside.get(veh)
                if prev_in is not None:
                    if (not prev_in) and ins and "entry_time_s" not in rec:
                        rec["entry_time_s"] = round(t, 3)
                        appr = last_normal.get(veh, lane if is_norm else "")
                        rec["entry_lane_id"] = appr
                        rec["entry_edge_id"] = appr.rsplit("_", 1)[0] if appr else ""
                    elif prev_in and (not ins) and "entry_time_s" in rec and "exit_time_s" not in rec:
                        rec["exit_time_s"] = round(t, 3)
                if "exit_time_s" in rec and "exit_lane_id" not in rec and is_norm:
                    rec["exit_lane_id"] = lane
                    rec["exit_edge_id"] = lane.rsplit("_", 1)[0]
                if is_norm:
                    last_normal[veh] = lane
                last_inside[veh] = ins

            # post-step dense dump: lane/edge mapped by SUMO (bulk)
            if DENSE and dense_this_tick:
                subs = traci.vehicle.getAllSubscriptionResults()
                for veh, st in sorted(dense_this_tick, key=lambda p: p[0]):
                    r = subs.get(veh, {})
                    scsv = st.get("sim_time_csv")
                    dense_w.writerow([
                        f"{t:.1f}", veh,
                        f"{st['sx']:.3f}", f"{st['sy']:.3f}",
                        r.get(VAR_LANE, ""), r.get(VAR_ROAD, ""),
                        st["lane_csv"],
                        f"{st['speed_csv']:.3f}", f"{st['angle']:.1f}",
                        dense_provenance(st),
                        (f"{scsv:.4f}" if scsv is not None else ""),
                        generation.get(veh, 0),
                    ])
                dense_rows_n += len(dense_this_tick)

            # post-step OCCUPANCY — ONLY on the logical tick (0.1 s), never in
            # the attach sub-steps (there it only re-commands; there is no new
            # state). getAllSubscriptionResults reads the local cache that arrived
            # with the step: zero additional TraCI round trips. Eligible:
            # REVEALED shadows (active, last_applied not None, outside
            # `parked`) — a parked ghost (park) is on a real lane
            # at v=0 and would count as stopped; the offnet staging falls out on its own
            # (empty lane) but is filtered anyway for uniformity.
            subs_post = (traci.vehicle.getAllSubscriptionResults()
                         if (OCC or VIEW) else None)   # local step cache, only once
            occ_counts = None
            if OCC:
                subs_occ = subs_post
                elegibles = [(veh, r.get(VAR_LANE, ""), r.get(VAR_SPEED, 0.0))
                             for veh, r in subs_occ.items()
                             if veh in active and veh not in parked
                             and last_applied.get(veh) is not None]
                occ_counts = occ_core.occupancy_counts(
                    elegibles, approach_lanes, occ_halt)
                for (brazo, carril), cnt in sorted(occ_counts.items()):
                    occ_w.writerow([f"{t:.1f}", brazo, carril,
                                    cnt["ocupacion"], cnt["detenidos"]])
                    occ_rows_n += 1
                if occ_probe_w is not None:
                    for veh in sorted(subs_occ):
                        r = subs_occ[veh]
                        estado = ("no_activo" if veh not in active
                                  else "parked" if veh in parked
                                  else "staging" if last_applied.get(veh) is None
                                  else "elegible")
                        occ_probe_w.writerow(
                            [f"{t:.1f}", veh, r.get(VAR_LANE, ""),
                             f"{r.get(VAR_SPEED, 0.0):.4f}", estado])

            # WS COUPLING POINT — ONLY on the logical tick, like the
            # occupancy: DecidedTickView of the DECIDED post-step state. It only READS
            # (zero TraCI commands, zero player mutation; passivity verified
            # byte by byte). The view does not retain references to the player's
            # state (blacklist verified by graph in tests). The emitter
            # consumes this variable (emit_step).
            if VIEW:
                tick_view = DecidedTickView.from_messages_tick(
                    seq=i, t_sim=sim_after, active=active, parked=parked,
                    last_applied=last_applied,
                    applied_ids={v for v, _ in applied_this_tick},
                    subs=subs_post, generation=generation,
                    assigned_type=assigned_type,
                    occupancy_counts=occ_counts if occ_counts is not None else {},
                    guard_report=store.guard_report())
                if ws_em is not None:
                    # REALTIME (pre-emission): the step with t_sim=t goes out at the
                    # wall instant (t_sim - t_sim0) from the first step.
                    # ABSOLUTE target against the anchor: no debt accumulation.
                    # Pure wall time post-decisions: the simulation and the log do
                    # not change (byte-compare in tests).
                    if WS_REALTIME:
                        if ws_rt_anchor is None:
                            ws_rt_anchor = (_ptime.monotonic(), sim_after)
                            # ANCHOR HANDOFF: the
                            # REALITY driver reads this anchor and sets its clock
                            # to R(w)=S(w), so that the visible lag is EXACTLY
                            # delay_s (1.0s) and not the ~0.48s that was compressed by
                            # sim_first (=0.4, first message) + startup jitter.
                            # sim_first is a PLAYER datum (perception), not the
                            # shell's: that is why the anchor travels by file, not by
                            # process epoch. Atomic write (write+replace):
                            # the driver never reads half. Only wall time, it touches
                            # no decision (passivity by byte-compare).
                            try:
                                _apath = os.path.join(args.out, "rt_anchor.txt")
                                with open(_apath + ".tmp", "w") as _af:
                                    _af.write(f"{ws_rt_anchor[0]:.6f} "
                                              f"{ws_rt_anchor[1]:.6f}\n")
                                os.replace(_apath + ".tmp", _apath)
                            except OSError:
                                pass   # best-effort: without the anchor the driver falls back to monotonic()
                        _target = ws_rt_anchor[0] + (sim_after - ws_rt_anchor[1])
                        _now = _ptime.monotonic()
                        if _now < _target:
                            ws_pace_sleep += _target - _now
                            _ptime.sleep(_target - _now)
                        ws_rt_last = (_ptime.monotonic(), sim_after)
                    # map -> canonicalize -> LOG -> enqueue (invariable order;
                    # the drop on Full only loses network, never the log)
                    ws_em.emit_step(tick_view)

            # sub-steps (attach only): re-command the last state so the
            # shadow stays still between ticks; no new decisions or logging
            for j in range(1, n_sub):
                present = set(traci.vehicle.getIDList())
                for veh in sorted(active):   # attach sub-steps: trace order
                    if veh not in present:
                        continue
                    st = last_applied.get(veh)
                    if st is None:
                        if not PARK:
                            moveToXY_traced(veh, STAGING_X, STAGING_Y, 0.0,
                                            "staging_sub")
                    else:
                        apply_moveToXY(veh, st)
                traci.simulationStep(round(t + j * args.server_step, 4))

            # termination: no messages to deliver and all removed
            if store.exhausted() and trackers and \
                    all(tr.retired for tr in trackers.values()):
                break
            i += 1
            if i - i_begin > HARD_CAP_TICKS:
                print("[AVISO] tope duro de ticks alcanzado; se corta el loop",
                      file=sys.stderr)
                break
    except CausalityViolation as e:
        guard_failure = str(e)
        print(f"\n[GUARDIA] VIOLACIÓN DE CAUSALIDAD — corrida abortada:\n{e}",
              file=sys.stderr)
    except traci.FatalTraCIError as e:
        # attach mode: the server (visual session) ended first; it closes
        # cleanly and writes the outputs of what was reproduced up to here
        print(f"\n[ATTACH] sesión SUMO terminada por el servidor: {e}",
              file=sys.stderr)
    finally:
        try:
            traci.close()
        except Exception:
            pass
        if dense_f is not None:
            dense_f.close()
        if occ_f is not None:
            occ_f.close()
        if occ_probe_f is not None:
            occ_probe_f.close()
        # WS emitter: end AFTER the last step (last line of the log) + close.
        # The verdict travels in 'end' and only there (contract Section 3.8): FAIL if the
        # guard aborted; otherwise the live status of the final guard_report.
        ws_report = None
        if ws_em is not None:
            g_fin = store.guard_report()
            ws_em.emit_end(guard_final=g_fin,
                           status=("FAIL" if guard_failure else g_fin["status"]),
                           detail=(guard_failure or ""))
            ws_report = ws_em.close()

    # --- salidas ---
    if val_rows:
        # CONTRACTUAL file order: tick asc, then vehicle_id
        # (lexicographic). Redundant with the loop's sorted iteration;
        # it guarantees the contract even if an internal iteration changes in the future.
        val_rows.sort(key=lambda r: (r["tick_i"], r["veh"]))
        val_path = os.path.join(args.out, "results", "validation_log.csv")
        with open(val_path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(val_rows[0].keys()))
            w.writeheader()
            w.writerows(val_rows)

    cross_cols = ["veh_id", "entry_time_s", "exit_time_s", "entry_lane_id",
                  "exit_lane_id", "entry_edge_id", "exit_edge_id"]

    def veh_sort_key(v):
        try:
            return (0, int(v.split("_")[-1]))
        except (ValueError, IndexError):
            return (1, v)

    with open(os.path.join(args.out, "results", "crossing_times.csv"), "w",
              newline="") as f:
        w = csv.DictWriter(f, fieldnames=cross_cols)
        w.writeheader()
        for veh in sorted(crossing, key=veh_sort_key):
            row = {"veh_id": veh}
            row.update(crossing[veh])
            w.writerow(row)

    n_ok = sum(1 for r in val_rows if r["projection_status"] == "OK")
    n_warn = sum(1 for r in val_rows if r["projection_status"] == "WARNING")
    n_fail = sum(1 for r in val_rows if r["projection_status"] == "FAIL")
    n_out = sum(1 for r in val_rows if r["projection_status"] == "FUERA_DE_RED")
    # the error stats cover what the net can represent; the raw max (including the
    # ticks past the end of dead-end lanes) is kept in error_pos_m_bruto_max
    errs = sorted(r["position_error_m"] for r in val_rows
                  if r["projection_status"] != "FUERA_DE_RED")
    err_bruto_max = max((r["position_error_m"] for r in val_rows), default=0.0)
    slot_rows = [r for r in val_rows if r["sim_time_csv"] is not None]
    delays = sorted(r["delay_efectivo_s"] for r in slot_rows)
    delay_err_max = max((abs(r["delay_efectivo_s"] - args.delay_s)
                         for r in slot_rows), default=0.0)

    def pct(a, p):
        return a[min(len(a) - 1, int(p * len(a)))] if a else 0.0

    prov_counts = Counter(r["provenance"] for r in val_rows)
    seg_counts = Counter(r["seg"] for r in val_rows
                         if r["provenance"] == "INTERPOLATED" and r["seg"])
    snap_events = [ev for tr in trackers.values() for ev in tr.snap_events]

    summary = {
        "modo": "causal_batch_con_guardia",
        "vehiculos_en_csv": len(trackers),
        "vehiculos_creados": len(assigned_type),
        "estados_aplicados": len(val_rows),
        "n_OK": n_ok, "n_WARNING": n_warn, "n_FAIL": n_fail,
        "n_FUERA_DE_RED": n_out,
        "error_pos_m_bruto_max": err_bruto_max,
        "error_pos_m": {"min": errs[0] if errs else 0.0, "mediana": pct(errs, 0.5),
                        "p95": pct(errs, 0.95), "max": errs[-1] if errs else 0.0},
        "delay_aplicado_s": args.delay_s,
        "delay_efectivo_s": {"min": delays[0] if delays else 0.0,
                             "mediana": pct(delays, 0.5),
                             "max": delays[-1] if delays else 0.0},
        "delay_error_s_max": round(delay_err_max, 4),
        "delay_ok": delay_err_max <= core.TICK / 2 + 1e-9,
        "held_total": sum(held_count.values()),
        "held_frames_causal": held_frames,
        "provenance_counts": dict(prov_counts),
        "interp_seg_counts": dict(seg_counts),
        "snap_events_n": len(snap_events),
        "snap_events": snap_events,
        "period_s": args.period_s,
        "buffer_s": args.delay_s,
        "lost_after_s": cfg.lost_after_s,
        "timeout_s": cfg.timeout_s,
        "guardia": store.guard_report(),
        "guardia_abort": guard_failure,
        "retiros_totales": sum(len(eps) for eps in retiro_report.values()),
        "reacquisiciones": reacquisitions,
        "retiro_report": retiro_report,
        "ruta_report": route_report,
        "creation_failures": creation_failures,
        "catchup_slots_saltados": dict(catchup_skipped),
        "colisiones_reportadas": collisions,
        "teleports": teleports,
        "vtype_inventario": vtype_usado,
        "vtype_desconocidos": vtype_desconocidos,
        "vtype_length_mismatches": length_mismatches,
        "angle_error_deg_max": max((abs(r["angle_error_deg"]) for r in val_rows),
                                   default=0.0),
        "transform": {"margin": args.margin, "xmin": xmin, "ymax": ymax},
        "ocupacion": ({"out": args.occupancy_out, "filas": occ_rows_n,
                       "halting_speed_mps": occ_halt,
                       "lane_map_md5": occ_lane_map_md5,
                       "probe_out": args.occupancy_probe_out or None}
                      if OCC else None),
        "ws_emitter": ({"log": args.ws_log, "port": args.ws_port or None,
                        "realtime": WS_REALTIME,
                        "pace_sleep_s": round(ws_pace_sleep, 3),
                        # realtime drift: (elapsed wall time) - (elapsed
                        # t_sim) between the first and last emitted step;
                        # the acid test requires |drift| <= quantization
                        "realtime_drift_s": (round(
                            (ws_rt_last[0] - ws_rt_anchor[0])
                            - (ws_rt_last[1] - ws_rt_anchor[1]), 4)
                            if (WS_REALTIME and ws_rt_anchor and ws_rt_last)
                            else None),
                        **ws_report} if ws_report is not None else None),
    }
    with open(os.path.join(args.out, "results", "summary.json"), "w") as f:
        json.dump(summary, f, indent=2)

    g = summary["guardia"]
    print()
    print("[GUARDIA]")
    print(f"modo                = {g['mode']}")
    print(f"accesos             = {g['accesses']}")
    print(f"max_lookahead_s     = {g['max_lookahead_s']}")
    print(f"violaciones         = {g['violations']}")
    print(f"status              = {g['status']}" +
          ("  (corrida ABORTADA por assert)" if guard_failure else ""))
    print()
    print("[SUMMARY]")
    print(f"vehiculos           = {len(trackers)} (creados {len(assigned_type)})")
    print(f"estados_aplicados   = {len(val_rows)}")
    print(f"OK/WARN/FAIL        = {n_ok}/{n_warn}/{n_fail}")
    print(f"fuera_de_red        = {n_out} (datos mas alla del final de carriles sin "
          f"salida; excluidos de OK/WARN/FAIL y del error)")
    print(f"position_error_max  = {errs[-1] if errs else 0.0:.4f}"
          f"  (bruto con fuera_de_red: {err_bruto_max:.4f})")
    print(f"delay_error_s_max   = {summary['delay_error_s_max']:.4f} "
          f"(ok={summary['delay_ok']})")
    print(f"teleports           = {teleports}")
    print(f"retiros_por_timeout = {summary['retiros_totales']} "
          f"(reacquisiciones {reacquisitions})")
    print(f"ruta: extensiones={route_report['extensiones_ok']} "
          f"diferidas={route_report['reintentos_diferidos']} "
          f"fallos={len(route_report['fallos'])}")
    for key in ["OBSERVED", "REACQUIRED", "REACQUIRED_SNAP", "INFERRED",
                "UNCERTAIN", "INTERPOLATED"]:
        if key in prov_counts:
            print(f"{key:20} = {prov_counts[key]}")
    print(f"snap_events         = {len(snap_events)}")
    print(f"held_frames_causal  = {held_frames}")
    if DENSE:
        print(f"dense_rows          = {dense_rows_n} -> {args.dense_out}")
    if OCC:
        print(f"ocupacion_filas     = {occ_rows_n} (halting<{occ_halt} m/s, "
              f"lane_map_md5={occ_lane_map_md5[:8]}) -> {args.occupancy_out}")
    print(f"out                 = {args.out}")

    if guard_failure:
        sys.exit(2)


if __name__ == "__main__":
    main()
