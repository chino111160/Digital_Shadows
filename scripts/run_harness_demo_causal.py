"""PERMANENT COMMAND — FULL CAUSAL SHADOW (the one the paper defends).

Shadow with no future information in the drawing (R1–R5 corrected): appearance at the stop-line
crossing, retirement by a "no update" counter (N=140), position without interpolating
across gaps (causal extrapolation + reconciliation on reappearance), route PREDICTED up to
the causal commit of the observed destination (exit stop-line crossing) and corrected there,
entry lane without using the observed exit. The CAUSALITY PROBE runs integrated with a
hard assert: if anything violates the rule, the command fails.

Usage:
  python scripts/run_harness_demo_causal.py                      # LIVE (ffplay+sumo-gui), segment
  python scripts/run_harness_demo_causal.py --video largo        # LIVE, long video
  python scripts/run_harness_demo_causal.py --split [--video V]  # split VIDEO|SHADOW mp4
  python scripts/run_harness_demo_causal.py --readout [--video V]# getLaneID/getPosition dump
  waiting boxes (live/split): ON by default; --no-waiting-boxes turns
  them off (pure causal shadow). --readout never draws them (measurement without contamination).

Implementation: imports the machinery in scripts/demo_causal/ (ChainCausalExit subclass
over the core). Its own outputs (verify_out/demo_causal/, datos/*_causal_completo_*).
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

AVISO = """
======================================================================
 SOMBRA CAUSAL COMPLETA (R1-R5) - sonda de causalidad integrada.
 Sistema honesto, sin informacion futura (sustituible por camara en vivo).
======================================================================
"""


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--video", choices=["tramo", "largo"], default="tramo")
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--split", action="store_true", help="renderiza el mp4 split en vez del vivo")
    mode.add_argument("--readout", action="store_true", help="volcado headless de lo dibujado")
    mode.add_argument("--3d", dest="web3d", action="store_true",
                      help="SOMBRA 3D WEB: sirve el stream de visión (tres fuentes) + ffplay del "
                           "video; imprime URL con gracia (un solo reloj contra el video)")
    ap.add_argument("--grace", type=float, default=20.0,
                    help="segundos para abrir la URL antes de arrancar (solo --3d)")
    ap.add_argument("--frames-3d", default=None,
                    help="con --split: dir de frames 3D pre-renderizados (step_<seq>.png, "
                         "seq = índice de frame) → añade el 3er panel SOMBRA 3D (video de "
                         "VISIÓN de 3 paneles). Se generan con replay_ws3d sobre el ws_log "
                         "de --3d (scene_vision).")
    ap.add_argument("--waiting-boxes", action=argparse.BooleanOptionalAction, default=True,
                    help="cajas de espera ancladas al carril en los modos VISUALES (default ON; "
                         "--no-waiting-boxes = sombra causal pura, sin revertir). "
                         "--readout NUNCA dibuja cajas (la medición no se contamina).")
    args = ap.parse_args()
    print(AVISO)
    os.environ.setdefault("SUMO_HOME", "/usr/share/sumo")
    os.environ.setdefault("DISPLAY", ":1")

    from paper_figures import common
    from demo_causal.run_harness_causal import (INPUTS, OUT_DIR, build_veh_causal,
                                                render_split, run_live, run_readout)
    try:
        stamp = common.check_provenance(INPUTS)
        print(f"[0] guardia de procedencia OK ({len(stamp)} insumos)")
    except RuntimeError as exc:
        # The md5 pins in paper_figures/common.py come from the authors' internal tree; the public
        # release has translated/anonymised configs and code, so the pins do not match. For the demo
        # this is only a warning; DS_STRICT_PROVENANCE=1 restores the abort (paper figures stay strict).
        if os.environ.get("DS_STRICT_PROVENANCE") == "1":
            raise
        print("[0] AVISO: guardia de procedencia no coincide con la versión pública (se continúa):")
        print("\n".join(str(exc).splitlines()[1:]))
        stamp = {rp: common.md5_of(rp) for rp in INPUTS}
    veh, chains, _commits, tid_root = build_veh_causal(args.video)
    wb_chains = chains if args.waiting_boxes else None
    n_corr = sum(1 for v in veh.values() if v["exit_obs"] and v["exit_obs"] != v["exit_pred"])
    print(f"    {len(veh)} vehículos causales; {n_corr} con corrección predicho→observado")

    if args.readout:
        run_readout(veh, args.video)
    elif args.split:
        render_split(veh, args.video, wb_chains, frames_3d=args.frames_3d)
        (OUT_DIR / f"provenance_demo_causal_{args.video}.txt").write_text("\n".join([
            f"demo CAUSAL COMPLETA ({args.video}) — comando permanente run_harness_demo_causal.py",
            *(f"  {p} = {m}" for p, m in sorted(stamp.items())),
        ]) + "\n")
    elif args.web3d:
        _run_web3d(args, veh, wb_chains, tid_root)
    else:
        run_live(veh, args.video, wb_chains, tid_root)


def _run_web3d(args, veh, wb_chains, tid_root) -> None:
    """--3d branch: serves the VISION stream to the browser. LAZY import of the emitter and
    the web — only this branch loads ws_emitter/websockets (the non-3D modes do not). A single
    clock against the video: run_live's pacing (T0+t_v) is the clock; ffplay plays
    the video externally. The video↔shadow offset is the INSTANTIATION LAG
    (~0.37 s, already in the HUD): the real reconstruction latency, a RESULT — it is NOT
    'synchronized' to zero."""
    import atexit
    import signal
    import subprocess

    import yaml

    from demo_causal.run_harness_causal import OUT_DIR, run_live, vision_hello_fields
    from ws_emitter.emitter import MessagesEmitter

    cfg = yaml.safe_load((ROOT / "config/sites/rusia/web3d.yaml").read_text())
    http_host = cfg["http_host"]
    http_port = int(os.environ.get("HTTP_PORT", cfg["vision_http_port"]))
    ws_port = int(os.environ.get("WS_PORT", cfg["vision_ws_port"]))
    scene = cfg["vision_scene"]          # VISION scene (its own net, distinct from messages)
    grace = float(os.environ.get("GRACE", args.grace))
    out = OUT_DIR / f"web3d_{args.video}"
    out.mkdir(parents=True, exist_ok=True)

    # static webserver (SEPARATE process, no TraCI): direct reuse of mode 5, with the
    # vision scene (its net_md5 = the hello's, otherwise the client rejects it).
    web = subprocess.Popen(
        [sys.executable, "-m", "ws_emitter.webserver", "--site", "rusia",
         "--http-port", str(http_port), "--ws-port", str(ws_port), "--scene", scene],
        cwd=str(ROOT))
    ws_em = None

    def _cleanup(*_a):
        try:
            web.terminate()
        except Exception:
            pass
        if ws_em is not None:
            try:
                ws_em.close()
            except Exception:
                pass
    atexit.register(_cleanup)
    for _s in (signal.SIGINT, signal.SIGTERM):
        signal.signal(_s, lambda *a: (_cleanup(), sys.exit(0)))

    time.sleep(1.0)
    print("=" * 62)
    print(f">>> SOMBRA 3D (VISIÓN):  http://{http_host}:{http_port}/index.html")
    print(f">>> tienes {grace:.0f}s (gracia) para abrirla antes de que arranque")
    print("=" * 62, flush=True)
    ws_em = MessagesEmitter(log_path=str(out / "ws_log.jsonl"),
                            hello_fields=vision_hello_fields(args.video),
                            ws_port=ws_port)
    try:
        time.sleep(grace)            # grace: the browser connects; ffplay+shadow start after this
        run_live(veh, args.video, wb_chains, tid_root, ws_em=ws_em)
    finally:
        _cleanup()


if __name__ == "__main__":
    main()
