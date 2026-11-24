# SUMO >= 1.18 crashes the vision pipeline (regression from 1.17.0 to 1.18.0)

Found on 2026-11-24 while trying to move the WSL demo to the latest SUMO. **Stay on SUMO 1.12.0** (the version the
README documents); this folder records why and keeps a minimal reproduction.

## Symptom

`python scripts/run_harness_demo_causal.py --readout` dies at `tr.simulationStep(t)`
(`scripts/demo_causal/run_harness_causal.py`, `run_readout`) with
`traci.exceptions.FatalTraCIError: Connection closed by SUMO.` The SUMO process exits with code 245
(= -11 mod 256, a likely SIGSEGV; the signal itself was not observed). It also happens headless, so it is not the GUI.
The V2I message shadow is not affected (it runs on 1.28.0 with the same metrics).

## Bisection (replay of the recorded session, deterministic)

| SUMO | `replay_crash.py` |
|---|---|
| 1.12.0, 1.14.1, 1.16.0, **1.17.0** | no crash |
| **1.18.0**, 1.20.0, 1.22.0, 1.24.0, 1.26.0, 1.27.0, 1.28.0 | crash after 981 commands |

(1.13, 1.15, 1.19, 1.21, 1.23, 1.25, 1.27.1 were not tested; PyPI `eclipse-sumo` 1.28.0 is the latest.)

## What the crash needs (and does not)

- Not caused by a TraCI call that fails: SUMO 1.28 prints a new error text for an unreachable `changeTarget`
  (`destination edge '-E6' unreachable`, vs 1.12 `No route found`), but pre-checking reachability so that call is never
  made does **not** avoid the crash (the re-created vehicle's `changeTarget` then succeeds and SUMO still dies on the next step).
- Needs the vehicles `v0` and `v12` of the recording alive together (v0 alone, v12 alone, v0+v13, v12+v13 do not crash) and the
  `remove`+`add` of `v0` followed by `moveToXY` into the junction (`keepRoute=1`).
- Disappears with `--no-internal-links true` (so it involves the junction's internal lanes; this changes the model and the
  lane ids the validation uses, so it is not a fix).
- Disappears with step lengths >= 0.05 s, but that changes the pipeline (it relies on `remove` taking effect before the
  `add` within 1 ms steps; on 1.12 it fails with "vehicle already exists"), so it is not a fix either.
- Not changed by `--collision.action warn|teleport|remove`, `--collision.check-junctions`, `--ignore-junction-blocker`,
  `--lanechange.duration 0`, `--device.rerouting.probability 0`.

## SUMO 1.17.0 is not a drop-in replacement either

On 1.17.0 the readout passes the causality probe (70 vehicles drawn, 66 effective re-emissions, 3 `remove+add` / 63 `changeTarget`,
same as 1.12.0) and every drawn lane is identical, but `exit_frames` differs for the 8 vehicles that exit to the south
(about 140 frames fewer on 1.17.0: 393 -> 253, 337 -> 197, ...). Until the cause is understood, results should be produced with 1.12.0.

## Reproduce

```bash
python -m venv v && v/bin/pip install eclipse-sumo==1.28.0 traci==1.28.0 sumolib==1.28.0
PATH=$PWD/v/bin:$PATH python docs/sumo_crash_1_18/replay_crash.py     # CRASH after 981 commands
```

`commands.json` was recorded by wrapping `traci.connection.Connection._sendCmd` while the readout ran on SUMO 1.28.0.
Not reported upstream yet.
