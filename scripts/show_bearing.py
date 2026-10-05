#!/usr/bin/env python3
"""Show bearing/yaw/estimate at a scenario's stall.

Usage: show_bearing.py <scenario> <t0> <t1>
"""
import json
import math
import os
import sys

D = os.path.expanduser("~/m20_chase/evidence/scenarios")
name, t0, t1 = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
rows = [json.loads(l) for l in open(os.path.join(D, f"{name}.jsonl"))]
for r in rows:
    if not (t0 <= r["t"] <= t1):
        continue
    d = r["dog"]
    e = r["est"]
    yaw = math.degrees(d[2])
    if e:
        brg = math.degrees(math.atan2(e[1], e[0]))
        rng = math.hypot(e[0], e[1])
        es = f"est=({e[0]:5.2f},{e[1]:5.2f}) brg={brg:6.1f} rng={rng:5.2f}"
    else:
        es = "est=None"
    c = r["cmd"]
    print(f"t={r['t']:6.1f} dog=({d[0]:5.2f},{d[1]:5.2f}) yaw={yaw:6.1f} {es} "
          f"cmd=({c[0]:5.2f},{c[1]:5.2f},{c[2]:5.2f}) {r['status']}")
