"""Deterministic replay of the TraCI session that crashes SUMO >= 1.18.

`commands.json` holds the 982 state-changing TraCI commands (vehicle.add/remove/setters, route.add,
simulationStep) that the vision shadow's `--readout` sends on the `tramo` segment, recorded just before
the SUMO process dies. Replaying them on the Rusia net reproduces the crash without the vision pipeline.

    SUMO <= 1.17.0 : "no crash after 982 commands"
    SUMO >= 1.18.0 : "CRASH after 981 commands" (SUMO exits with 245 = -11 mod 256, i.e. a likely
                     SIGSEGV; the connection is closed during the last simulationStep)

Usage (the `sumo` binary and `traci` must come from the SUMO version under test):
    python docs/sumo_crash_1_18/replay_crash.py                          # as recorded
    EXTRA="--no-internal-links true" python docs/sumo_crash_1_18/replay_crash.py   # no crash (changes the model)
"""
import json
import os
import sys
from pathlib import Path

import traci

HERE = Path(__file__).resolve().parent
NET = HERE.parents[1] / "config/sites/rusia/sumo/intersection_rusia.net.xml"
commands = json.loads((HERE / "commands.json").read_text())

cmd = (["sumo", "-n", str(NET)] + os.environ.get("EXTRA", "").split()
       + ["--step-length", "0.001", "--no-step-log", "true", "--no-warnings", "true",
          "--collision.action", "none", "--time-to-teleport", "-1", "--ignore-route-errors", "true"])
traci.start(cmd)
conn = traci.getConnection()
print("TraCI/SUMO version:", traci.getVersion()[1])

sent = 0
try:
    for cmd_id, var_id, obj_id, fmt, values in commands:
        try:
            conn._sendCmd(cmd_id, var_id, obj_id, fmt, *values)
        except traci.TraCIException:
            pass          # a handled TraCI error (e.g. changeTarget unreachable) is part of the recording
        sent += 1
    print(f"no crash after {sent} commands")
    code = 0
except traci.FatalTraCIError as exc:
    print(f"CRASH after {sent} commands: {exc}")
    code = 1
finally:
    try:
        traci.close()
    except Exception:
        pass
sys.exit(code)
