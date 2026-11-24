#!/usr/bin/env bash
# Runs the minimal reproduction with the `sumo` found in PATH. Prints the version and the exit code.
#   exit 0   : simulation finished (SUMO <= 1.17.0)
#   exit 139 : segmentation fault (SUMO >= 1.18.0)
cd "$(dirname "$0")"
sumo --version | head -1
sumo -n intersection_rusia.net.xml -r crash.rou.xml -e 60 > /dev/null 2>&1
echo "exit code: $?"
