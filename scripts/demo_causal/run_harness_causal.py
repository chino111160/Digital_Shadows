"""IMPLEMENTATION of the permanent command `scripts/run_harness_demo_causal.py` (R1+R2+R3+R4+R5).

(The user entry point is run_harness_demo_causal.py; this module is the implementation
detail.)

Lifecycle and drawing WITHOUT information later than the decision frame:
- R1 appearance: from the stop-line crossing (incremental cross_t of the core).
- R2 retirement: literal counter of "time without update" (searchsorted over the past),
  N = lost_track_buffer_frames = 140 (~4.73 s, M3; = end commit of the core).
- R3 position: interpolation ONLY between contiguous samples (≤2·DT); over larger gaps,
  causal extrapolation over the path at the last arc-length velocity (GapFiller
  semantics); when the observation reappears, an honest jump to the real one. Post-end: frozen.
- R4 route: from the origin to the causal COMMIT of the observed destination (exit stop-line
  crossing, detected incrementally by ChainCausalExit — subclass, core intact) the vehicle goes
  along the PREDICTED route; at the commit the connection/target is re-chosen and the recon continues over the
  observed path (the correction jump where predicted≠observed is the real behavior).
- R5 departLane: at instantiation, connection with PREDICTED exit and exit_carril=None; the
  turn-observed group disambiguation also drops (v0 goes to the 2-3 mean: causal truth).

FULL PROBE (hard assert): appearance ≥ own crossing · retirement by counter · route with no
change before its observed commit · ZERO interpolations across gaps >2·DT
(the extrapolations are counted). PASS mandatory or the render fails.

Usage:
  python scripts/demo_causal/run_harness_causal.py --video tramo|largo [--live|--readout]
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
from paper_figures import common  # noqa: E402
from demo_causal.causal_core import CausalOrchestratorExit  # noqa: E402
from digital_shadow.config.site import load_site_config  # noqa: E402

VID_W, VID_H, SUMO_W, HUD_H = 1200, 646, 1148, 64
# dt/fps from the site config (fps_ref EXACT as a Fraction, e.g. "355/12"):
# float(1/fps_ref) is bit-identical to 12/355 (0x1.14e9a52355d06p-5). FPS only
# feeds ffmpeg's -r (with .3f it produces the string "29.583").
FPS_REF = load_site_config("rusia").fps_ref
FPS = float(FPS_REF)
DT = float(1 / FPS_REF)
GAP_DT = 2 * DT               # threshold: beyond it there is NO interpolation (R3)
# TraCI constants for the post-step readback: LITERAL values; they are checked
# against traci.constants in _setup (SUMO_HOME is not set when the module is
# imported, so traci is not imported at the top).
VAR_POSITION, VAR_ANGLE, VAR_LANE_ID, VAR_ROAD_ID, VAR_TYPE = 0x42, 0x43, 0x51, 0x50, 0x4f
_DRAWN_VARS = [VAR_POSITION, VAR_ANGLE, VAR_LANE_ID, VAR_ROAD_ID, VAR_TYPE]
# drawn vtype -> contract vclass (inverse of VTYPE from run_harness_viz). The
# caller resolves vclass from the POST-STEP getTypeID (what is REVEALED), NOT from veh[vid]["cls"].
_VTYPE_VCLASS = {"viz_car": "car", "viz_bus": "bus", "viz_truck": "truck"}
# VISION provenance (v3 contract Section 3.5, sumo subset): branch of causal_pos.
PROV_OBSERVED, PROV_INTERPOLATED, PROV_EXTRAPOLATED, PROV_HELD = (
    "OBSERVED", "INTERPOLATED", "EXTRAPOLATED", "HELD")
from collections import namedtuple  # noqa: E402
# DECIDED state of a frame box: flat scalars, no reference to
# structures with a future. anchor comes from the SAME drawing branch
# (carril → lane; else → raw_homography).
BoxState = namedtuple("BoxState",
                      "tid cx cy ang anchor carril length width cls prov")
COCO_VCLASS = {2: "car", 5: "bus", 7: "truck"}   # COCO id → contract vclass
# handoff marker: CLOSED keys of the v3 contract handoff section,
# provenance=HELD, WITHOUT a prospective field (no ETA, no duration).
HandoffMark = namedtuple("HandoffMark", "id source last_pos last_t vclass prov")
OUT_DIR = ROOT / "verify_out" / "demo_causal"
SHOTS = Path(os.environ.get("SHOTS_DIR", "/tmp/gemelo_rvh_scratch")) / "causal_shots"

VIDEOS = {
    "tramo": ROOT / "data/sources/rusia/rusia_tramo_000-180.mp4",
    "largo": ROOT / "data/sources/rusia_long/rusia_long_cfr355-12.mp4",
}
DATA = {
    "tramo": ("verify_out/tracks_rusia.csv", "verify_out/events/rusia/track_events.csv"),
    "largo": ("verify_out/generalizacion_long/tracks_long.csv",
              "verify_out/generalizacion_long/track_events_long.csv"),
}
INPUTS = [
    "digital_shadow/sumo/reconstruct.py", "digital_shadow/perception/detector.py",
    "config/sites/rusia/sumo/intersection_rusia.net.xml", "config/sites/rusia/site.yaml",
    "config/sites/rusia/zones.yaml", "config/sites/rusia/sumo/lane_map.yaml",
    "config/sites/rusia/lane_turn_rule.yaml", "config/sites/rusia/turns.csv",
    "scripts/run_harness_viz.py",
]


# --------------------------------------------------------------------------- #
# Causal core (subclass) → veh with predicted + causal observed commit
# --------------------------------------------------------------------------- #
def build_veh_causal(which: str, tracks_path=None, events_path=None):
    # tracks_path/events_path (optional, ABSOLUTE): truncated tracks + regenerated
    # events. None -> the site's (DATA[which]). Additive: the production
    # callers pass nothing and see the behavior intact.
    import derive_track_events as dte
    import reconnect_fragments as rf
    from digital_shadow.config.site import load_site_config
    from digital_shadow.geometry.ground_contact import add_ground_contact
    from digital_shadow.harness.sink import RecordingSink
    from digital_shadow.sumo.lane_map import load_lane_map
    from digital_shadow.sumo.position import load_alpha, load_homography, project_to_world
    from digital_shadow.sumo.route import load_lane_turn_rule, load_turns, predict_turn
    from digital_shadow.tracking.causal_reconnect import CausalReconnector
    from audit_harness_part3 import internal_shapes_from_net
    from run_harness_viz import COCO_NAME

    tpath, epath = DATA[which]
    if tracks_path is None and which == "largo" and not (
            (ROOT / tpath).exists() and (ROOT / epath).exists()):
        # bootstrap: if the largo derivatives are missing, they are generated with the stages inherited
        # from the generalization script (CFR-conditioned source + canonical tracking + events)
        from run_harness_demo_long import condition_source, run_events, run_tracking
        print("    [bootstrap largo] derivados ausentes → generándolos (CFR + tracking + eventos)…")
        condition_source(); run_tracking(); run_events()
    site = load_site_config("rusia")
    center = np.array([site.frame_width / 2.0, site.frame_height / 2.0])
    tracks = pd.read_csv(tracks_path or (ROOT / tpath)); add_ground_contact(tracks)
    base = pd.read_csv(events_path or (ROOT / epath))
    excluded = set(tracks.track_id.unique()) - set(base.track_id.unique())
    feats = rf.track_features(tracks, center, dte.CORE_RADIUS_PX, dte.WINDOW_FRAMES)
    lm = load_lane_map("rusia")
    H = load_homography("rusia")
    shapes = internal_shapes_from_net(lm, ROOT / "config/sites/rusia/sumo/intersection_rusia.net.xml")
    rule = load_lane_turn_rule("rusia"); turns = load_turns("rusia")
    orch = CausalOrchestratorExit(
        tracks=tracks, feats=feats,
        reconnector=CausalReconnector(buffer_frames=site.reid_temporal_window_frames,
                                      corridor_px=site.reid_spatial_corridor_px,
                                      valid_moves=rf.load_valid_movements("rusia")),
        lane_map=lm, H=H, rule=rule, turns=turns,
        stop_lines=dte.load_stop_lines("rusia"), lane_bands=dte.load_lane_bands("rusia"),
        center=center, alpha=load_alpha(), buffer_frames=site.reid_temporal_window_frames,
        excluded=excluded, internal_shapes=shapes, frame_dt=1 / FPS_REF)
    commits, chains = orch.run(RecordingSink())
    # {track_id → root}: flat scalars, NO reference to chains nor to any
    # structure with a future. It is re-id identity, not trajectory.
    tid_root = {int(k): int(v) for k, v in orch.track_root.items()}

    inst_t = {c.chain: c.t_commit for c in commits if c.kind == "instanciacion"}
    veh = {}
    for ch in chains.values():
        if not ch.instantiated:
            continue
        world = np.array([project_to_world(H, (gx, gy)) for gx, gy in zip(ch.gx, ch.gy)])
        ocommit = next((c for c in ch.route.commits() if c.field == "origin"), None)
        lane_label = str(ocommit.lane_label) if ocommit else ""
        group = [int(p) for p in lane_label.split("-")] \
            if all(p.isdigit() for p in lane_label.split("-") if p) and lane_label else [1]
        # R4/R5: PREDICTED destination at instantiation (carril→turn; arm_only → through)
        pt = predict_turn(rule, ch.entry_arm, lane_label) if ch.entry_arm in lm.arms else None
        pred = turns.get((ch.entry_arm, pt)) if pt else None
        if pred is None and ch.entry_arm in lm.arms:
            pred = turns.get((ch.entry_arm, "through"))
        veh[ch.vid] = dict(
            t=np.array(ch.ts), xy=world, cross_t=ch.cross_t,
            delay=max(0.1, inst_t.get(ch.root, (ch.cross_t or 0) + 0.37) - (ch.cross_t or 0.0)),
            cls=COCO_NAME.get(ch.running_class(), "car"),
            entry_arm=ch.entry_arm, lane_label=lane_label, group_carriles=group,
            entry_edge=lm.arms[ch.entry_arm].entry_edge if ch.entry_arm in lm.arms else None,
            exit_pred=pred, exit_obs=ch.exit_obs_arm, t_exit_commit=ch.t_exit_commit,
            exit_carril_obs=ch.exit_carril_obs,
            exit_edge_pred=lm.arms[pred].exit_edge if pred in lm.arms else None,
            exit_edge_obs=lm.arms[ch.exit_obs_arm].exit_edge if ch.exit_obs_arm in lm.arms else None)
    return veh, chains, commits, tid_root


# --------------------------------------------------------------------------- #
# Reconstruction in TWO segments (predicted → observed at the commit) + arclens
# --------------------------------------------------------------------------- #
def _arc_of(path):
    return np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(path, axis=0).T))])


def _sample_arclens(path, arc, pts):
    s, j = np.empty(len(pts)), 0
    for k, p in enumerate(pts):
        while j + 1 < len(path) and np.hypot(*(path[j + 1] - p)) <= np.hypot(*(path[j] - p)):
            j += 1
        s[k] = arc[j]
    return s


def _point_at(path, arc, s):
    s = float(np.clip(s, 0.0, arc[-1]))
    return np.array([np.interp(s, arc, path[:, 0]), np.interp(s, arc, path[:, 1])])


def _nearest_arclen(path, arc, p):
    d = np.hypot(path[:, 0] - p[0], path[:, 1] - p[1])
    return float(arc[int(np.argmin(d))])


def prepare_recon_causal(veh, tr):
    from digital_shadow.config.site import load_site_config
    from digital_shadow.sumo.lane_map import load_lane_map
    from digital_shadow.sumo.position import load_homography
    from digital_shadow.sumo.presentation import load_presentation, presentation_smooth
    from digital_shadow.sumo.reconstruct import build_path, choose_connection, reconstruct_world
    lm = load_lane_map("rusia")
    H_pres = load_homography("rusia")
    deadband_px, median_window_s = load_presentation("rusia")
    for v in veh.values():
        v["depart_lane"], v["segs"] = 0, []
        ok_pred = v["entry_arm"] in lm.arms and v["exit_pred"] in lm.arms
        try:
            if not ok_pred:
                raise ValueError("movimiento predicho no resoluble")
            # R5: connection at instantiation WITHOUT observed exit (full group, no disambiguation)
            conn_p = choose_connection(tr, lm, v["entry_arm"], v["exit_pred"],
                                       v["group_carriles"], exit_carril=None)
            if conn_p is not None:
                v["depart_lane"] = int(conn_p["from_idx"])
            path_p = build_path(tr, lm, v["entry_arm"], v["exit_pred"], v["group_carriles"],
                                exit_carril=None, conn=conn_p)
            tc = v["t_exit_commit"]
            k = int(np.searchsorted(v["t"], tc, side="right")) if tc is not None else len(v["t"])
            parts = []
            if k > 0:
                rt1, rxy1 = reconstruct_world(v["xy"][:k], v["t"][:k], path_p)
                parts.append((np.asarray(rt1), np.asarray(rxy1, float), path_p))
            if tc is not None and k < len(v["t"]) and v["exit_obs"] in lm.arms:
                conn_o = choose_connection(tr, lm, v["entry_arm"], v["exit_obs"],
                                           v["group_carriles"], exit_carril=v["exit_carril_obs"])
                path_o = build_path(tr, lm, v["entry_arm"], v["exit_obs"], v["group_carriles"],
                                    exit_carril=v["exit_carril_obs"], conn=conn_o)
                rt2, rxy2 = reconstruct_world(v["xy"][k:], v["t"][k:], path_o)
                parts.append((np.asarray(rt2), np.asarray(rxy2, float), path_o))
            rt = np.concatenate([p[0] for p in parts])
            rxy = np.vstack([p[1] for p in parts])
            lo = 0
            for prt, pxy, pth in parts:
                arc = _arc_of(pth)
                v["segs"].append(dict(lo=lo, hi=lo + len(prt), path=pth, arc=arc,
                                      s=_sample_arclens(pth, arc, pxy)))
                lo += len(prt)
        except Exception:
            rt, rxy = np.asarray(v["t"]), np.asarray(v["xy"], float)   # raw (no path)
            v["segs"] = []
        v["rt"] = rt
        v["draw"] = presentation_smooth(rxy, rt, H_pres, deadband_px, median_window_s)


# --------------------------------------------------------------------------- #
# CAUSAL position sampler (R3) + probe
# --------------------------------------------------------------------------- #
class SondaTotal:
    def __init__(self, veh, timeout_s):
        self.veh, self.timeout_s = veh, timeout_s
        self.first_last = {vid: [None, None] for vid in veh}
        self.switch_t = {}
        self.reemit = {}          # vid -> {t, destino, metodo, efectiva} (R4 route re-emission)
        self.n_extrap = 0
        self.max_interp_gap = 0.0

    def verify(self):
        bad = []
        for vid, (a, b) in self.first_last.items():
            if a is None:
                continue
            v = self.veh[vid]
            if v["cross_t"] is None or a < v["cross_t"] - 1e-6:
                bad.append(f"{vid}: dibujado {a:.2f} antes de su cruce {v['cross_t']}")
            if b > float(v["t"][-1]) + self.timeout_s + 1e-6:
                bad.append(f"{vid}: dibujado {b:.2f} tras última-obs+timeout")
        for vid, ts_ in self.switch_t.items():
            tc = self.veh[vid]["t_exit_commit"]
            if tc is None or ts_ < tc - 1e-6:
                bad.append(f"{vid}: cambio de ruta en {ts_:.2f} antes de su commit {tc}")
        # EXTENDED PROBE (R4 re-emission): no re-emission before the commit, and the destination
        # used is EXACTLY the one of the causal commit (not the tail of the full track).
        for vid, r in self.reemit.items():
            tc = self.veh[vid]["t_exit_commit"]
            if tc is None or r["t"] < tc - 1e-6:
                bad.append(f"{vid}: re-emisión en {r['t']:.2f} antes del commit {tc}")
            if r["destino"] != self.veh[vid]["exit_obs"]:
                bad.append(f"{vid}: re-emisión con destino {r['destino']} ≠ commit "
                           f"{self.veh[vid]['exit_obs']}")
            if not r["efectiva"]:
                bad.append(f"{vid}: re-emisión NO efectiva (ruta sin el egreso observado)")
        if self.max_interp_gap > GAP_DT + 1e-9:
            bad.append(f"interpolación a través de hueco {self.max_interp_gap:.3f}s > 2·DT")
        if bad:
            raise AssertionError("SONDA TOTAL FALLÓ:\n" + "\n".join(bad))
        n = sum(1 for a in self.first_last.values() if a[0] is not None)
        met = {}
        for r in self.reemit.values():
            met[r["metodo"]] = met.get(r["metodo"], 0) + 1
        print(f"[sonda TOTAL] PASS: {n} dibujados; aparición≥cruce, retiro por contador, "
              f"ruta sin cambio pre-commit, re-emisiones {sum(met.values())} efectivas "
              f"con destino=commit ({met}), 0 interp a través de huecos "
              f"(extrapolaciones causales: {self.n_extrap} draws)")


def causal_pos(v, tshift, sonda: SondaTotal):
    """Position decided at `tshift` + its PROVENANCE (Section 3.5). The position is
    BYTE FOR BYTE that of the readout (byte-compare); prov is an additional datum: the branch
    taken. OBSERVED = tshift on a reconstruction node (w≈0); INTERPOLATED =
    between nodes with a gap ≤ GAP_DT; EXTRAPOLATED = causal extrapolation over the
    path (n_extrap); HELD = frozen with no path."""
    ts = v["rt"]
    i = int(np.searchsorted(ts, tshift, side="right")) - 1
    if i < 0:
        return None, None
    if i + 1 < len(ts) and (ts[i + 1] - ts[i]) <= GAP_DT:
        sonda.max_interp_gap = max(sonda.max_interp_gap, float(ts[i + 1] - ts[i]))
        w = (tshift - ts[i]) / (ts[i + 1] - ts[i])
        pos = (1 - w) * v["draw"][i] + w * v["draw"][i + 1]
        prov = PROV_OBSERVED if w <= 1e-9 else PROV_INTERPOLATED   # w≈0: on a node
        return pos, prov
    # GAP or post-end → causal extrapolation over the path (or frozen if there is no path)
    sonda.n_extrap += 1
    seg = next((s_ for s_ in v["segs"] if s_["lo"] <= i < s_["hi"]), None)
    if seg is None:
        return np.asarray(v["draw"][i], float), PROV_HELD          # raw: freeze
    j0 = max(seg["lo"], i - 3)
    dt0 = float(ts[i] - ts[j0])
    vel = max(0.0, (seg["s"][i - seg["lo"]] - seg["s"][j0 - seg["lo"]]) / dt0) if dt0 > 0 else 0.0
    tc = v["t_exit_commit"]
    last_seg = v["segs"][-1]
    if tc is not None and ts[i] < tc <= tshift and seg is not last_seg:
        # the commit falls inside the gap: advance on predicted up to the commit and re-project
        p_c = _point_at(seg["path"], seg["arc"], seg["s"][i - seg["lo"]] + vel * (tc - ts[i]))
        s_o = _nearest_arclen(last_seg["path"], last_seg["arc"], p_c)
        return _point_at(last_seg["path"], last_seg["arc"], s_o + vel * (tshift - tc)), PROV_EXTRAPOLATED
    return _point_at(seg["path"], seg["arc"], seg["s"][i - seg["lo"]] + vel * (tshift - ts[i])), PROV_EXTRAPOLATED


# --------------------------------------------------------------------------- #
# Per-frame placement (lifecycle R1+R2, route R4/R5)
# --------------------------------------------------------------------------- #
def _sub_drawn(tr, vid):
    """Subscribes the drawn vehicle to the 4 variables of the post-step readback
    (pos_plane + heading + carril + edge of what is REVEALED). Read-only: it commands
    nothing → it does not alter the dynamics (byte-compare of the readout)."""
    try:
        tr.vehicle.subscribe(vid, _DRAWN_VARS)
    except Exception:
        pass


def place_causal(stage, tr, veh, live, sonda: SondaTotal, t, VTYPE, prov_out=None):
    # prov_out (optional, PURE write): dict {vid: provenance} DECIDED this frame,
    # for the vision emitter. None by default → run_readout and the other callers
    # intact (byte-compare). It does not read the future: only the branch of causal_pos.
    active = []
    for vid, v in veh.items():
        tshift = t - v["delay"]
        ts = v["t"]
        i = int(np.searchsorted(ts, tshift, side="right")) - 1
        if i < 0 or v["cross_t"] is None or tshift < v["cross_t"]:
            continue                                             # R1: does not exist yet
        if tshift - float(ts[i]) > sonda.timeout_s:              # R2: causal counter
            if vid in live:
                try:
                    stage.remove(vid)
                except Exception:
                    pass
                live.pop(vid, None)
            continue
        active.append(v["delay"])
        if vid not in live:
            if v["entry_edge"] is None:
                continue
            stage.add(vid, v["entry_edge"], v.get("depart_lane", 0), 5.0,
                      vtype=VTYPE.get(v["cls"], "DEFAULT_VEHTYPE"))
            tgt0 = v["exit_edge_pred"]
            if tgt0:
                suc = next((l[0].rsplit("_", 1)[0]
                            for l in tr.lane.getLinks(tgt0 + "_0")), None)
                v["_tgt"] = suc or tgt0
                try:
                    stage.set_target(vid, v["_tgt"])
                except Exception:
                    pass
            _sub_drawn(tr, vid)                  # readback post-step
            live[vid] = True
        if vid in live:
            # R4: target change ONLY at the causal commit of the observed destination.
            # changeTarget from an internal lane fails silently and leaves the vehicle
            # anchored to the PREDICTED route (it would not draw its real turn). That is why
            # effectiveness is VERIFIED and, if the route does not contain the observed egress, the
            # vehicle is re-created with the commit route (remove+add; ALL the values come from the
            # causal commit — the probe asserts it). The moveToXY of this same frame
            # repositions it: no extra jump.
            if (v["t_exit_commit"] is not None and vid not in sonda.switch_t
                    and tshift >= v["t_exit_commit"] and v["exit_edge_obs"]):
                suc = next((l[0].rsplit("_", 1)[0]
                            for l in tr.lane.getLinks(v["exit_edge_obs"] + "_0")), None)
                v["_tgt"] = suc or v["exit_edge_obs"]
                metodo = "changeTarget"
                try:
                    stage.set_target(vid, v["_tgt"])
                    efectiva = v["exit_edge_obs"] in tr.vehicle.getRoute(vid) \
                        or v["_tgt"] in tr.vehicle.getRoute(vid)
                except Exception:
                    efectiva = False
                if not efectiva:
                    metodo = "remove+add"
                    try:
                        stage.remove(vid)
                    except Exception:
                        pass
                    try:
                        stage.add(vid, v["entry_edge"], v.get("depart_lane", 0), 5.0,
                                  vtype=VTYPE.get(v["cls"], "DEFAULT_VEHTYPE"))
                        stage.set_target(vid, v["_tgt"])
                        _sub_drawn(tr, vid)      # re-add loses the subscription: re-subscribe
                        efectiva = v["exit_edge_obs"] in tr.vehicle.getRoute(vid) \
                            or v["_tgt"] in tr.vehicle.getRoute(vid)
                    except Exception:
                        live.pop(vid, None)
                        continue
                sonda.switch_t[vid] = tshift
                sonda.reemit[vid] = dict(t=tshift, destino=v["exit_obs"], metodo=metodo,
                                         efectiva=efectiva)
            p, prov = causal_pos(v, tshift, sonda)
            if p is None:
                continue
            if prov_out is not None:
                prov_out[vid] = prov      # decided provenance (pure write)
            a = sonda.first_last[vid]
            a[0] = tshift if a[0] is None else a[0]
            a[1] = tshift
            try:
                tr.vehicle.moveToXY(vid, "", -1, float(p[0]), float(p[1]), keepRoute=1)
                if v.get("_tgt") and len(tr.vehicle.getRoute(vid)) < 2:
                    try:
                        tr.vehicle.changeTarget(vid, v["_tgt"])
                    except Exception:
                        pass
                tr.vehicle.setSpeed(vid, 0.0)
            except Exception:
                live.pop(vid, None)
    return active


def hud(canvas, t, delays):
    W = canvas.shape[1]
    cv2.rectangle(canvas, (0, 0), (W, HUD_H), (12, 12, 12), -1)
    # DESCRIPTIVE labels: causality is demonstrated in the probe and the text, it is not
    # declared in a banner. The data line shows the MEASURED LAG.
    cv2.putText(canvas, "VIDEO", (14, 26), cv2.FONT_HERSHEY_SIMPLEX,
                0.66, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(canvas, "SOMBRA EN SUMO",
                (VID_W + 14, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.66, (120, 255, 120), 2, cv2.LINE_AA)
    if delays:
        d = np.array(delays)
        linea = (f"t_video = {t:6.2f} s    en escena: {len(delays):2d}    "
                 f"rezago de instanciacion: mediana {np.median(d):.2f} s  "
                 f"[{d.min():.2f}-{d.max():.2f} s]")
    else:
        linea = f"t_video = {t:6.2f} s    en escena:  0"
    cv2.putText(canvas, linea,
                (14, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (170, 255, 170), 1, cv2.LINE_AA)
    cv2.line(canvas, (VID_W, 0), (VID_W, canvas.shape[0]), (12, 12, 12), 3)


def hud3(canvas, t, delays):
    """3-panel HUD (VIDEO | VISION SHADOW | SHADOW 3D) — same visual style as the
    MESSAGES video (white/cyan/green labels + data line). VISION semantics:
    the line shows the MEASURED INSTANTIATION LAG (median + range), not the messages
    buffer. Causality is shown as measured, it is not declared in a banner."""
    F, AA = cv2.FONT_HERSHEY_SIMPLEX, cv2.LINE_AA
    cv2.rectangle(canvas, (0, 0), (canvas.shape[1], HUD_H), (14, 14, 14), -1)
    cv2.putText(canvas, "VIDEO (realidad)", (14, 26), F, 0.72, (255, 255, 255), 2, AA)
    cv2.putText(canvas, "SOMBRA VISION (SUMO)", (VID_W + 14, 26), F, 0.72, (90, 220, 255), 2, AA)
    cv2.putText(canvas, "SOMBRA 3D", (VID_W + SUMO_W + 14, 26), F, 0.72, (170, 255, 170), 2, AA)
    if delays:
        d = np.array(delays)
        linea = (f"t_video = {t:6.2f} s    en escena: {len(delays):2d}    "
                 f"rezago de instanciacion: mediana {np.median(d):.2f} s  "
                 f"[{d.min():.2f}-{d.max():.2f} s]")
    else:
        linea = f"t_video = {t:6.2f} s    en escena:  0"
    cv2.putText(canvas, linea, (14, 52), F, 0.56, (200, 200, 200), 1, AA)
    for x in (VID_W, VID_W + SUMO_W):
        cv2.line(canvas, (x, 0), (x, canvas.shape[0]), (40, 40, 40), 2)


def _setup(veh):
    from digital_shadow.sumo.stage import SumoStage
    from run_harness_viz import VTYPE, define_vtypes
    from digital_shadow.config.site import load_site_config
    timeout_s = load_site_config("rusia").lost_track_buffer_frames * DT
    stage = SumoStage(ROOT / "config/sites/rusia/sumo/intersection_rusia.net.xml",
                      gui=True, step_length=0.001).start()
    tr = stage._traci
    define_vtypes(tr)
    import traci.constants as _C   # runtime check (SUMO already up)
    assert (VAR_POSITION, VAR_ANGLE, VAR_LANE_ID, VAR_ROAD_ID, VAR_TYPE) == (
        _C.VAR_POSITION, _C.VAR_ANGLE, _C.VAR_LANE_ID, _C.VAR_ROAD_ID, _C.VAR_TYPE), \
        "constantes TraCI del readback divergen de traci.constants en el entorno"
    tr.gui.setSchema("View #0", "real world")
    tr.gui.setZoom("View #0", 700)
    tr.gui.setOffset("View #0", 57.0, 74.4)
    prepare_recon_causal(veh, tr)
    return stage, tr, SondaTotal(veh, timeout_s), VTYPE



class WaitingBoxLayer:
    """TraCI glue for the WAITING BOXES (--waiting-boxes; ON by default at the
    run_harness_demo_causal.py entry point, OFF only in the direct internal invocation
    and in --readout, which never draws boxes; causal PRESENTATION layer, NOT measurement
    — pure logic and contract in digital_shadow/occupancy/waiting_boxes.py). Dimension
    and color are read from the REAL vTypes at runtime (vehicletype.get*): exact identity
    with the causal vehicle. Overlap in a dense queue: it is drawn as is."""
    LAYER = 4                                     # above the roadway, below the vehicles
    COCO_VTYPE = {2: "viz_car", 5: "viz_bus", 7: "viz_truck"}

    def __init__(self, tr, which, chains):
        import yaml
        import derive_track_events as dte
        from digital_shadow.geometry.ground_contact import add_ground_contact
        from digital_shadow.occupancy.waiting_boxes import (
            FILL_ALPHA, WaitingBoxTracker, anchor_to_lane, box_corners,
            lane_angle_towards_stop, lane_axis_from_shape)
        from digital_shadow.sumo.position import (AlphaBetaFilter, load_alpha,
                                                   longitudinal_pos)
        from digital_shadow.occupancy.zones import (classify_points,
                                                     load_approach_zones,
                                                     occupancy_counts)
        from digital_shadow.sumo.lane_map import load_lane_map
        from digital_shadow.sumo.position import load_homography, project_to_world
        self.tr = tr
        self._corners = box_corners
        tracks = pd.read_csv(ROOT / DATA[which][0]); add_ground_contact(tracks)
        zones = load_approach_zones("rusia")
        excl, _, _ = dte.load_exclusion_polygons("rusia")
        # RAW points per frame for the c2-sur rule: the "c1-sur empty" condition is
        # queried with LITERAL occupancy_counts (single source with the metric — an
        # equivalent precomputed value would be a second code path that could diverge
        # in staleness)
        self._zones = zones
        self._excl = [np.asarray(q, float) for q in excl.values()]
        self._counts = occupancy_counts
        self._raw_by_frame = {int(f): g[["gx", "gy"]].to_numpy(float)
                              for f, g in tracks.groupby("frame_index")}
        self._occ_cache = (-1, None)   # FULL pixel_zones occupancy per frame
        # SINGLE SOURCE of membership: the same classify_points that occupancy_counts
        # aggregates — c2-sur doctrine extended to the boxes: zero
        # parallel zone paths.
        pts = tracks[["gx", "gy"]].to_numpy(float)
        arm_col, lane_col = classify_points(pts, zones, self._excl)
        sel = arm_col != ""
        tracks = tracks[sel].copy()
        tracks["arm"] = arm_col[sel]
        tracks["carril"] = lane_col[sel]
        H = load_homography("rusia")
        world = np.array([project_to_world(H, (gx, gy))
                          for gx, gy in tracks[["gx", "gy"]].to_numpy(float)])
        tracks["wx"], tracks["wy"] = world[:, 0], world[:, 1]
        self.by_frame = {int(f): g[["track_id", "wx", "wy", "class_id", "arm", "carril"]]
                         .to_numpy(object) for f, g in tracks.groupby("frame_index")}
        lm = load_lane_map("rusia")
        self.angle = {arm: lane_angle_towards_stop(tr.lane.getShape(ae.entry_edge + "_0"))
                      for arm, ae in lm.arms.items()}
        # axis and angle PER LANE (anchoring per the position.py doctrine — lateral to the
        # lane center, homographic longitudinal via longitudinal_pos; straight lines verified)
        self.lane_axis, self.lane_angle = {}, {}
        for arm, ae in lm.arms.items():
            for k, idx in ae.entry_lanes.items():
                shape = tr.lane.getShape(f"{ae.entry_edge}_{idx}")
                self.lane_axis[(arm, int(k))] = lane_axis_from_shape(shape)
                self.lane_angle[(arm, int(k))] = lane_angle_towards_stop(shape)
        self._anchor = anchor_to_lane
        self._AlphaBeta, self._longpos = AlphaBetaFilter, longitudinal_pos
        self._alpha_cfg = load_alpha()                     # causal filter (config, like vehicles)
        self._filters: dict[int, object] = {}
        with open(ROOT / "config/sites/rusia/approach_zones.yaml", encoding="utf-8") as fh:
            mcfg = (yaml.safe_load(fh) or {}).get("measurement") or {}
        cross_t = {int(ch.root): ch.cross_t for ch in chains.values() if ch.cross_t is not None}
        self.trk = WaitingBoxTracker(float(mcfg.get("theta_s", 0.0)),
                                     float(mcfg.get("merge_gap_s", 0.0)), cross_t, DT)
        self.dims = {v: (tr.vehicletype.getLength(v), tr.vehicletype.getWidth(v),
                         tuple(int(c) for c in tr.vehicletype.getColor(v)[:3]))
                     for v in set(self.COCO_VTYPE.values())}
        self.alpha = FILL_ALPHA
        self.arm_of: dict[int, str] = {}
        self.active: set[str] = set()
        self.decided_boxes: dict = {}      # decided boxes of the last frame

    def occupancy_counts(self, fi: int):
        """FULL pixel_zones occupancy of frame `fi` (ALL zones), MEMOIZED:
        a SINGLE call to occupancy_counts per frame, shared with _sur_c1_empty
        (c2-sur doctrine: single source, zero parallel paths that diverge in
        staleness). It is a RAW COUNT of contacts per zone/lane, NOT the paper
        metric (Section 3.4/Section 3.7). {} if the frame has no raw points."""
        if self._occ_cache[0] != fi:
            pts = self._raw_by_frame.get(fi)
            counts = (self._counts(pts, self._zones, self._excl)
                      if pts is not None and len(pts) else {})
            self._occ_cache = (fi, counts)
        return self._occ_cache[1]

    def _sur_c1_empty(self, fi: int) -> bool:
        """Is the c1-sur zone free of any contact in frame fi? — reads from the
        MEMOIZED full occupancy (same function, same data, same index ["sur"]["lanes"][1]:
        identical result, no second code path)."""
        counts = self.occupancy_counts(fi)
        n = counts["sur"]["lanes"][1] if counts else 0
        return n == 0

    def draw(self, t):
        fi = int(round(t / DT))
        self.decided_boxes = {}            # rebuilt each frame
        present = []
        for tid, wx, wy, cls, arm, carril in self.by_frame.get(fi, ()):
            tid = int(tid)
            self.arm_of.setdefault(tid, str(arm))
            carril = int(carril) if int(carril) > 0 else None
            # c2-sur rule (presentation ONLY; the metric/GT stay sin_carril): the south
            # background with no lane zone → anchors to c2 IF c1-sur is empty AT ASSIGNMENT (sticky
            # by vote while the box lives).
            if (carril is None and arm == "sur" and not self.trk.has_lane_votes(tid)
                    and self._sur_c1_empty(fi)):
                carril = 2
            present.append((tid, float(wx), float(wy), int(cls), carril))
        present_tids = {p[0] for p in present}   # OBSERVED vs HELD (coasting)
        vis = self.trk.update(t, present)
        want = {f"wb_{tid}": (tid, v) for tid, v in vis.items()}
        for pid in self.active - set(want):
            try:
                self.tr.polygon.remove(pid)
            except Exception:
                pass
        for gone in set(self._filters) - {tid for tid, _ in want.values()}:
            self._filters.pop(gone, None)                  # the filter dies with the box
        drawn = set()
        for pid, (tid, (wx, wy, cls, carril)) in want.items():
            vt = self.COCO_VTYPE.get(cls, "viz_car")
            length, width, rgb = self.dims[vt]
            arm = self.arm_of[tid]
            if carril is not None:
                # ANCHORING: homographic longitudinal projected onto the axis + causal filter;
                # lateral fixed to the center of the voted lane.
                axis = self.lane_axis[(arm, carril)]
                flt = self._filters.get(tid)
                if flt is None:
                    flt = self._filters[tid] = self._AlphaBeta(self._alpha_cfg)
                flt.observe(t, self._longpos(axis, (wx, wy)))
                pos = flt.value()
                cx = axis.origin[0] + pos * axis.direction[0]
                cy = axis.origin[1] + pos * axis.direction[1]
                ang = self.lane_angle[(arm, carril)]
            else:
                # sin_carril background: no lane to anchor to → raw homography + approach
                # angle (DECLARED fallback, not dressed up)
                cx, cy, ang = wx, wy, self.angle[arm]
            shape = [tuple(q) for q in self._corners((cx, cy), ang, length, width)]
            try:
                if pid in self.active:
                    self.tr.polygon.setShape(pid, shape)
                else:
                    self.tr.polygon.add(pid, shape, (*rgb, self.alpha), fill=True,
                                        polygonType="waiting_box", layer=self.LAYER)
                drawn.add(pid)
                # RETAIN the decided state of the box. anchor from the same branch;
                # prov OBSERVED/HELD (coasting).
                anchor = "lane" if carril is not None else "raw_homography"
                prov = PROV_OBSERVED if tid in present_tids else PROV_HELD
                self.decided_boxes[tid] = BoxState(
                    tid, float(cx), float(cy), float(ang), anchor, carril,
                    float(length), float(width), int(cls), prov)
            except Exception:
                pass
        self.active = drawn


class HandoffRegistry:
    """OBSERVED box→vehicle bridge (source `handoff`, v3 contract). A HELD marker
    at the last known position of the box, between the box going extinct and
    the reconstructed vehicle appearing.

    NON-NEGOTIABLE CAUSAL RULE — OBSERVED events only; it NEVER consults
    delay/cross_t/inst_t (the handoff does not know how long it lasts, as it would not know live):
      · IS BORN from `boxes_prev ∖ boxes_now` (the root's box stopped being at the
               wb output THIS frame) and its vid is not yet in `live`.
      · DIES from: `vid ∈ live` (success: the vehicle took over) / the root
               reappears in `boxes_now` (cancellation: transient extinction) / it exceeds
               `handoff_max_hold_s` (PRESENTATION cap from config, not prediction).

    CLOSED keys of the contract: {id, source, last_pos(+t), vclass}, provenance=HELD,
    no ETA. `last_pos` = last OBSERVED position of the box. The map {tid→root} is
    scalars (no reference to chains)."""

    def __init__(self, tid_root: dict, max_hold_s: float):
        self.tid_root = {int(k): int(v) for k, v in tid_root.items()}
        self.max_hold = float(max_hold_s)
        self.pending: dict = {}      # root -> dict(last_pos, last_t, vclass, born)
        self.last_box: dict = {}     # root -> (cx, cy, vclass, t) last OBSERVED box
        self.prev_roots: set = set()

    def update(self, t, decided_boxes, live_ids):
        roots_now = set()
        for tid, bs in decided_boxes.items():
            root = self.tid_root.get(int(tid))
            if root is None:
                continue
            roots_now.add(root)
            vclass = COCO_VCLASS.get(int(bs.cls), "car")
            self.last_box[root] = (bs.cx, bs.cy, vclass, t)   # snapshot of what was observed
        # BIRTH: box extinct (observed) and vid not yet alive
        for root in self.prev_roots - roots_now:
            vid = f"v{root}"
            if vid not in live_ids and root in self.last_box and root not in self.pending:
                cx, cy, vclass, lt = self.last_box[root]
                self.pending[root] = dict(last_pos=(cx, cy), last_t=lt,
                                          vclass=vclass, born=t)
        # DEATH (three events, in order: success, cancellation, presentation cap)
        for root in list(self.pending):
            if f"v{root}" in live_ids:
                del self.pending[root]                    # success
            elif root in roots_now:
                del self.pending[root]                    # cancellation (box reappears)
            elif t - self.pending[root]["born"] > self.max_hold:
                del self.pending[root]                    # PRESENTATION cap
        self.prev_roots = roots_now
        return [HandoffMark(id=f"v{root}", source="handoff",
                            last_pos=m["last_pos"], last_t=m["last_t"],
                            vclass=m["vclass"], prov=PROV_HELD)
                for root, m in sorted(self.pending.items())]


def render_split(veh, which: str, wb_chains=None, frames_3d=None):
    """VIDEO | SUMO-SHADOW [| SHADOW 3D]. With frames_3d (dir of step_<seq>.png, where
    seq = video frame index, the same as in run_live) it adds the third 3D panel
    pre-rendered from the VISION ws_log and uses the 3-panel MESSAGES-style HUD.
    Without frames_3d: 2 panels (original behavior). The SUMO panel and the 3D come from the
    SAME reconstruction (build_veh_causal) → the vehicles match across panels."""
    import bisect
    from pathlib import Path as _P
    stage, tr, sonda, VTYPE = _setup(veh)
    wb = WaitingBoxLayer(tr, which, wb_chains) if wb_chains is not None else None
    SHOTS.mkdir(parents=True, exist_ok=True)
    for f in SHOTS.glob("*.png"):
        f.unlink()
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    # pre-rendered 3D: {seq: path}; seq = fi (same frame index as this loop). The
    # 3D may come subsampled (e.g. 10 Hz) → for each fi the nearest seq is taken.
    d3d, seqs3d = {}, []
    if frames_3d:
        d3d = {int(p.stem.split("_")[1]): p for p in _P(frames_3d).glob("step_*.png")}
        seqs3d = sorted(d3d)
        if not seqs3d:
            raise SystemExit(f"sin frames step_*.png en {frames_3d}")
    P3D = SUMO_W if frames_3d else 0
    out = OUT_DIR / (f"demo_vision_3panel_{which}.mp4" if frames_3d
                     else f"demo_causal_{which}.mp4")
    cap = cv2.VideoCapture(str(VIDEOS[which]))
    n_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    cw, chh = VID_W + SUMO_W + P3D, VID_H + HUD_H
    ff = subprocess.Popen(["ffmpeg", "-y", "-f", "rawvideo", "-pix_fmt", "bgr24",
                           "-s", f"{cw}x{chh}", "-r", f"{FPS:.3f}", "-i", "-",
                           "-c:v", "libx264", "-crf", "18", "-preset", "medium",
                           "-pix_fmt", "yuv420p", str(out)], stdin=subprocess.PIPE,
                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    live, written = {}, 0
    last_3d = np.zeros((VID_H, SUMO_W, 3), np.uint8)
    for fi in range(n_frames):
        ok, vid_frame = cap.read()
        if not ok:
            break
        t = fi * DT
        if wb:
            wb.draw(t)
        active = place_causal(stage, tr, veh, live, sonda, t, VTYPE)
        tr.simulationStep(t)
        shot = SHOTS / f"s_{fi % 8:d}.png"
        tr.gui.screenshot("View #0", str(shot), width=SUMO_W, height=VID_H)
        tr.simulationStep(t + 0.001)
        sumo_img = cv2.imread(str(shot))
        if sumo_img is None:
            continue
        if sumo_img.shape[:2] != (VID_H, SUMO_W):
            sumo_img = cv2.resize(sumo_img, (SUMO_W, VID_H))
        canvas = np.zeros((chh, cw, 3), np.uint8)
        canvas[HUD_H:, :VID_W] = vid_frame
        canvas[HUD_H:, VID_W:VID_W + SUMO_W] = sumo_img
        if frames_3d:
            i = bisect.bisect_left(seqs3d, fi)          # 3D of the seq closest to fi
            if i >= len(seqs3d):
                i = len(seqs3d) - 1
            elif i > 0 and (seqs3d[i] - fi) > (fi - seqs3d[i - 1]):
                i -= 1
            img3 = cv2.imread(str(d3d[seqs3d[i]]))
            if img3 is not None:
                if img3.shape[:2] != (VID_H, SUMO_W):
                    img3 = cv2.resize(img3, (SUMO_W, VID_H))
                last_3d = img3
            canvas[HUD_H:, VID_W + SUMO_W:] = last_3d    # missing frame → repeat the last
            hud3(canvas, t, active)
        else:
            hud(canvas, t, active)
        ff.stdin.write(canvas.tobytes())
        written += 1
        if written % 1000 == 0:
            print(f"    {written} frames ({t:.1f}s)…", flush=True)
    ff.stdin.close(); ff.wait(); stage.close(); cap.release()
    sonda.verify()
    print(f"[ok] {out.relative_to(ROOT)} ({written} frames)")


def _pixel_occ(counts):
    """{zone:{lanes:{k:count}}} from occupancy_counts -> scalar {zone:{k:count}}."""
    return {str(z): {str(k): int(n) for k, n in (v.get("lanes") or {}).items()}
            for z, v in (counts or {}).items()}


def build_vision_view(*, seq, t_sim, drawn, prov_out, decided_boxes, tid_root,
                      marks, occ_counts, sonda):
    """Caller of the VISION emitter: assembles the DecidedTickView of the THREE sources from
    what is DECIDED/OBSERVED in the frame (post-step readback, decided boxes, handoff
    registry, raw occupancy), NEVER from veh[vid]. Feeders = scalars. vclass
    from the POST-STEP getTypeID (VAR_TYPE) — NOT veh[vid]["cls"] (the messages hole).
    canonical id f"v{root}" via tid_root (identity); box cls via COCO_VCLASS."""
    from ws_emitter.view import DecidedTickView          # lazy (modes without 3D do not load web)
    sumo_subs = {vid: {VAR_POSITION: r[VAR_POSITION], VAR_ANGLE: r[VAR_ANGLE]}
                 for vid, r in drawn.items()
                 if VAR_POSITION in r and VAR_ANGLE in r}
    sumo_vclass = {vid: _VTYPE_VCLASS.get(r.get(VAR_TYPE), "car")
                   for vid, r in drawn.items()}
    boxes = [(f"v{tid_root[int(tid)]}", bs.prov, bs.cx, bs.cy, bs.ang, bs.anchor,
              bs.carril, COCO_VCLASS.get(int(bs.cls), "car"))
             for tid, bs in decided_boxes.items() if int(tid) in tid_root]
    handoffs = [(m.id, m.last_pos[0], m.last_pos[1], m.last_t, m.vclass) for m in marks]
    guard = {"n_extrap": int(sonda.n_extrap),
             "max_interp_gap": round(float(sonda.max_interp_gap), 4)}
    return DecidedTickView.from_vision_tick(
        seq=seq, t_sim=t_sim, sumo_subs=sumo_subs, sumo_prov=dict(prov_out),
        sumo_vclass=sumo_vclass, boxes=boxes, handoffs=handoffs,
        occupancy_pixel=_pixel_occ(occ_counts), guard_report=guard)


def vision_hello_fields(which):
    """Fields of the VISION shadow hello (Section 3.1). shadow=vision, layers with
    CONSTITUTIVE vision_box, occupancy pixel_zones, frame_geometry null (demo base).
    dims from the vTypes (single source); md5 of network and tracks."""
    import hashlib
    from run_harness_viz import VTYPE, VTYPE_DEF
    net = ROOT / "config/sites/rusia/sumo/intersection_rusia.net.xml"
    tracks = ROOT / DATA[which][0]

    def _md5(p):
        return hashlib.md5(Path(p).read_bytes()).hexdigest()
    dims = {vc: {"length_m": VTYPE_DEF[vt][2], "width_m": VTYPE_DEF[vt][3]}
            for vc, vt in VTYPE.items()}
    return {
        "shadow": "vision", "mode": "replay", "dt": DT, "site": "rusia",
        "frame_geometry": None,                          # demo base: no bbox_px
        "net_md5": _md5(net), "artifact_md5": {"tracks_csv": _md5(tracks)},
        "guard_mode": "assert",
        "layers": ["sumo", "vision_box", "handoff"],     # vision_box CONSTITUTIVE
        "occupancy_semantics": {"kind": "pixel_zones", "semantics": "conteo_crudo"},
        "vclass_dims": dims,
    }


def run_live(veh, which: str, wb_chains=None, tid_root=None, ws_em=None):
    from digital_shadow.sumo.presentation import load_handoff_max_hold
    stage, tr, sonda, VTYPE = _setup(veh)
    wb = WaitingBoxLayer(tr, which, wb_chains) if wb_chains is not None else None
    # handoff registry: only if there are boxes AND the {tid→root} map.
    handoff = (HandoffRegistry(tid_root, load_handoff_max_hold("rusia"))
               if (wb is not None and tid_root is not None) else None)
    if ws_em is not None:
        ws_em.emit_hello()                               # first line, BEFORE the loop
    try:
        proc = subprocess.Popen(["ffplay", "-an", "-autoexit", "-loglevel", "quiet",
                                 "-window_title", "VIDEO (realidad)", "-x", "900",
                                 str(VIDEOS[which])], stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL)
    except FileNotFoundError:
        # ffplay not installed (e.g. bare WSL): the shadow still runs; play the video separately.
        print(f"[aviso] ffplay no encontrado: se omite la ventana de video ({VIDEOS[which]})",
              flush=True)
        proc = None
    time.sleep(0.6)
    cap = cv2.VideoCapture(str(VIDEOS[which]))
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)); cap.release()
    live = {}
    marks = []
    T0 = time.perf_counter()
    try:
        for fi in range(n):
            t_v = fi * DT
            if wb:
                wb.draw(t_v)
            prov_out = {}                                # {vid: provenance} decided
            place_causal(stage, tr, veh, live, sonda, t_v, VTYPE, prov_out=prov_out)
            tr.simulationStep(t_v)
            # post-step readback: what is REVEALED, by SUBSCRIPTION (incl. VAR_TYPE).
            drawn = tr.vehicle.getAllSubscriptionResults()
            if handoff is not None:                      # dies from "vid ∈ live" = present post-step
                marks = handoff.update(t_v, wb.decided_boxes, set(drawn))
            # A SINGLE CLOCK against the video: pace and emit at the scheduled instant
            target = T0 + t_v
            now = time.perf_counter()
            if now < target:
                time.sleep(target - now)
            if ws_em is not None and wb is not None:
                ws_em.emit_step(build_vision_view(
                    seq=fi, t_sim=t_v, drawn=drawn, prov_out=prov_out,
                    decided_boxes=wb.decided_boxes, tid_root=(tid_root or {}),
                    marks=marks, occ_counts=wb.occupancy_counts(fi), sonda=sonda))
        if ws_em is not None:
            ws_em.emit_end(guard_final={"n_extrap": int(sonda.n_extrap)}, status="PASS")
        sonda.verify()
    finally:                                             # trap: do not leave ffplay/SUMO hanging
        try:
            stage.close()
        except Exception:
            pass
        try:
            if proc is not None:
                proc.terminate()
        except Exception:
            pass


def run_readout(veh, which: str):
    """Headless dump of what is DRAWN (getRoadID/getLaneID/getPosition after step) under the
    full causal placement → *_causal_completo.csv (the pre-causal one is kept separately)."""
    from collections import Counter
    from digital_shadow.sumo.lane_map import load_lane_map
    stage, tr, sonda, VTYPE = _setup(veh)
    lm = load_lane_map("rusia")
    t_end = max(float(v["t"][-1]) + v["delay"] for v in veh.values()) + sonda.timeout_s
    rows, live = [], {}
    for fi in range(int(t_end / DT) + 2):
        t = fi * DT
        place_causal(stage, tr, veh, live, sonda, t, VTYPE)
        tr.simulationStep(t)
        for vid in list(live):
            try:
                rows.append((vid, round(t, 4), tr.vehicle.getRoadID(vid),
                             tr.vehicle.getLaneID(vid), *[round(c, 3) for c in tr.vehicle.getPosition(vid)]))
            except Exception:
                live.pop(vid, None)
    stage.close()
    sonda.verify()
    df = pd.DataFrame(rows, columns=["vid", "t", "road", "lane", "x", "y"])
    dat = ROOT / "verify_out/imagenes_finales/datos"
    dat.mkdir(parents=True, exist_ok=True)    # not versioned in the public release
    df.to_csv(dat / f"causal_sumo_readout_causal_completo_{which}.csv", index=False)
    inv = {}
    for arm, ae in lm.arms.items():
        inv[("entry", ae.entry_edge)] = {i: c for c, i in ae.entry_lanes.items()}
        inv[("exit", ae.exit_edge)] = {i: c for c, i in ae.exit_lanes.items()}
    summary = []
    for vid, g in df.groupby("vid"):
        v = veh[vid]
        rec = {"vid": vid, "entry_arm": v["entry_arm"], "exit_arm_obs": v["exit_obs"]}
        for marco, edge in (("entry", v["entry_edge"]), ("exit", v["exit_edge_obs"] or v["exit_edge_pred"])):
            sub = g[g.road == edge]
            carril = None
            if len(sub):
                idx = Counter(int(l.rsplit("_", 1)[1]) for l in sub.lane).most_common(1)[0][0]
                carril = inv.get((marco, edge), {}).get(idx)
            rec[f"{marco}_carril_drawn"] = carril
            rec[f"{marco}_frames"] = len(sub)
        summary.append(rec)
    out = dat / f"causal_sumo_lanes_causal_completo_{which}.csv"
    pd.DataFrame(summary).to_csv(out, index=False)
    print(f"[ok] {out.relative_to(ROOT)} ({len(summary)} vehículos)")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--video", choices=["tramo", "largo"], default="tramo")
    ap.add_argument("--live", action="store_true")
    ap.add_argument("--readout", action="store_true")
    ap.add_argument("--waiting-boxes", action="store_true",
                    help="cajas de espera (presentación causal). Default OFF SOLO en esta "
                         "invocación interna directa; el comando de usuario "
                         "(run_harness_demo_causal.py) las activa por defecto")
    args = ap.parse_args()
    os.environ.setdefault("SUMO_HOME", "/usr/share/sumo")
    os.environ.setdefault("DISPLAY", ":1")
    stamp = common.check_provenance(INPUTS)
    print(f"[0] guardia OK ({len(stamp)} insumos)")
    veh, _chains, _commits, tid_root = build_veh_causal(args.video)
    n_corr = sum(1 for v in veh.values() if v["exit_obs"] and v["exit_obs"] != v["exit_pred"])
    print(f"    {len(veh)} vehículos; {n_corr} con corrección predicho→observado")
    wb_chains = _chains if args.waiting_boxes else None
    if args.live:
        run_live(veh, args.video, wb_chains, tid_root)
    elif args.readout:
        run_readout(veh, args.video)
    else:
        render_split(veh, args.video, wb_chains)
        (OUT_DIR / f"provenance_demo_causal_{args.video}.txt").write_text("\n".join([
            f"demo CAUSAL COMPLETA ({args.video}) — R1+R2+R3+R4+R5 en la variante (subclase)",
            "núcleo de producción intacto; commit destino observado al cruce del pare de salida",
            *(f"  {p} = {m}" for p, m in sorted(stamp.items())),
        ]) + "\n")


if __name__ == "__main__":
    main()
