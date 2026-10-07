#!/usr/bin/env python3
"""Show the node's own safety numbers over a time window of a scenario.

Usage: show_state.py <scenario> [t0] [t1]
"""
import json
import os
import sys

D = os.path.expanduser("~/m20_chase/evidence/scenarios")
name = sys.argv[1]
t0 = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
t1 = float(sys.argv[3]) if len(sys.argv) > 3 else 1e9

rows = [json.loads(l) for l in open(os.path.join(D, f"{name}.jsonl"))]
print(f"{'t':>6}  {'cmd(vx,vy,wz)':>22}  status            state")
for r in rows:
    if not (t0 <= r["t"] <= t1):
        continue
    cmd = r.get("cmd") or [0, 0, 0]
    st = r.get("state") or r.get("fsm") or "-"
    print(f"{r['t']:6.1f}  ({cmd[0]:5.2f},{cmd[1]:5.2f},{cmd[2]:5.2f})  "
          f"{r['status']:<17} {st}")
