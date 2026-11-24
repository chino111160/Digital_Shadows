# Draft issue for https://github.com/eclipse-sumo/sumo/issues (NOT filed)

**Title:** Segmentation fault since 1.18.0 (works in 1.17.0): two vehicles with conflicting routes crossing a junction with internal links

## Problem description

`sumo` crashes with a segmentation fault (exit code 139) on a tiny network with two vehicles whose routes
`E1 -E3` and `E1 -E6` cross the same junction. It does not need TraCI, sublane, a traffic-light program, or any
non-default option. The same command finishes normally on SUMO 1.12.0 to 1.17.0 and crashes on every release from
1.18.0 to 1.28.0 (all tested, see table). With `--no-internal-links true` it does not crash.

## Steps to reproduce

Files (attached): `intersection_rusia.net.xml` (18 KB), `crash.rou.xml` (7 lines).

```
sumo -n intersection_rusia.net.xml -r crash.rou.xml -e 60
echo $?        # 139 on >= 1.18.0, 0 on <= 1.17.0
```

`crash.rou.xml`:

```xml
<routes>
  <route id="rA" edges="E1 -E3"/>
  <route id="rB" edges="E1 -E6"/>
  <vehicle id="v0"  route="rB" depart="0" departLane="2" departPos="5"  departSpeed="0"/>
  <vehicle id="v12" route="rA" depart="1" departLane="2" departPos="30" departSpeed="0"/>
</routes>
```

## Results by version

Official PyPI wheels (`eclipse-sumo`), Ubuntu 26.04 on WSL2 x86-64, running the `sumo` binary of each wheel directly.

| SUMO | exit code |
|---|---|
| 1.12.0, 1.13.0, 1.14.1, 1.15.0, 1.16.0, **1.17.0** | 0 |
| **1.18.0**, 1.19.0, 1.20.0, 1.21.0, 1.22.0, 1.23.1, 1.24.0, 1.25.0, 1.26.0, 1.27.0, 1.27.1, 1.28.0 | 139 (Segmentation fault) |

Not tested: a build from git HEAD, a debug build. No backtrace is available (no gdb in this environment), and the
last output line of `--verbose` is `Simulation version 1.28.0 started with time: 0.00.`

## What I found about the trigger

Stepping through TraCI (only to read the state, the crash is the same without it), the last state before the crash is

```
t=  4.0  v0 E1_2@17.31 v=6.12 | v12 E1_2@35.67 v=3.73
t=  5.0  v0 E1_2@24.89 v=7.58 | v12 :J5_9_1@3.25 v=5.62      <- crash while advancing to t=6
```

i.e. `v12` (to `-E3`) is already on the junction-internal lane `:J5_9_1` while `v0` (to `-E6`, a conflicting link)
approaches on `E1_2`.

- Does **not** crash: only one of the two vehicles; both vehicles on the same route; both vehicles on lane 0 or on lane 1;
  `--no-internal-links true`.
- Still crashes with: `--tls.all-off true`, `--step-length 0.1`, `--lateral-resolution 0.8`, `--collision.action none`,
  `--default.speeddev 0`, `--sloppy-insert true`, `--ignore-junction-blocker 0`, `--time-to-teleport -1`.
- It is sensitive to the insertion state: without `departSpeed="0"` the same two vehicles do not crash, and
  `v12 depart="5"` or `departPos="10"` do not crash.
- Re-generating the junction with `netconvert` (dropping edges, or removing the traffic light) makes the crash disappear,
  so it depends on the exact internal-lane layout of this junction.

## Context

Found while running a thesis project (a traffic "digital shadow" driven through TraCI), where the crash first appeared as
`Connection closed by SUMO` after a `remove`/`add` and `moveToXY`. Bisecting that 982-command TraCI session led to this
file-only reproduction. I did not find an existing issue for it; #13994 (also 1.18.0) involves the sublane model and
`moveToXY` with `keepRoute=3`, which this does not.

## Expected behavior

The simulation runs to the end (or reports a collision / error), as in 1.17.0.
