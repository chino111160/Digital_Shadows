# SUMO >= 1.18 segfaults on this project's network (regression from 1.17.0 to 1.18.0)

Found on 2026-11-24 while trying to move the WSL demo to the latest SUMO. **Stay on SUMO 1.12.0** (the version the
README documents). This folder holds a minimal, file-only reproduction and a draft issue for SUMO.

| File | What |
|---|---|
| `crash.rou.xml` | two vehicles, routes `E1 -E3` and `E1 -E6`, both on lane 2 |
| `intersection_rusia.net.xml` | the Rusia network (copy of `config/sites/rusia/sumo/`, 18 KB) |
| `run_repro.sh` | runs `sumo -n intersection_rusia.net.xml -r crash.rou.xml -e 60` and prints the exit code |
| `ISSUE.md` | working draft of the issue filed upstream: <https://github.com/eclipse-sumo/sumo/issues/19197> |

## Result

`run_repro.sh` exits 0 on SUMO 1.12.0, 1.13.0, 1.14.1, 1.15.0, 1.16.0 and **1.17.0**, and with **139 (SIGSEGV)** on
**1.18.0**, 1.19.0, 1.20.0, 1.21.0, 1.22.0, 1.23.1, 1.24.0, 1.25.0, 1.26.0, 1.27.0, 1.27.1 and 1.28.0 (the latest on PyPI).
No TraCI is needed. In the demo it shows up as `Connection closed by SUMO` / exit 245 (= -11 mod 256 through the
pip wrapper of `sumo`; the real binary returns 139).

## How it was reduced

1. The vision shadow's `--readout` died at `simulationStep`. A recording of its 982 state-changing TraCI commands
   reproduced the crash deterministically without the vision pipeline (crash from 1.18.0 on).
2. Delta debugging (ddmin) cut it to 9 commands: two vehicles added on `E1` lane 2, `changeTarget` of `v12` to `-E3` and
   of `v0` to `-E6`, one `moveToXY`, one step.
3. Re-sequencing (insert each vehicle with its own step) showed `moveToXY` is not needed, and explicit routes in a
   `.rou.xml` reproduce it with the plain `sumo` binary.

Earlier hypotheses that were wrong: the crash is **not** caused by a failing `changeTarget`, by the vehicle types, by the
step length, nor by the traffic light.

## What it depends on

- Both vehicles, on different conflicting links of the same junction (`v12` to `-E3`, `v0` to `-E6`) and both on lane 2
  of `E1`; the last state before the crash has `v12` on the internal lane `:J5_9_1` and `v0` approaching on `E1_2`.
- Junction internal links: `--no-internal-links true` avoids it (but changes the model, so it is not a fix here).
- Not changed by `--tls.all-off`, `--step-length`, `--lateral-resolution`, `--collision.action`, `--sloppy-insert`,
  `--ignore-junction-blocker`, `--default.speeddev`.
- Sensitive to the insertion state (`departSpeed="0"` present, `v12` depart 1 s, `departPos` 30) and to the exact junction
  geometry: regenerating the junction with `netconvert` makes it disappear.

## Impact on this project

The vision shadow (`--readout`) needs SUMO <= 1.17.0. SUMO 1.17.0 is not a drop-in replacement: its readout draws the same
lanes but `exit_frames` differs for 8 vehicles that exit to the south (about 140 frames fewer), so results are produced
with 1.12.0. The V2I message shadow works on 1.28.0.

Reported upstream on 2026-11-24 as [eclipse-sumo/sumo#19197](https://github.com/eclipse-sumo/sumo/issues/19197); no existing issue was found
(#13994 is also 1.18.0 but is a sublane / `moveToXY keepRoute=3` crash). Check it for a fix before moving past SUMO 1.12.0.
