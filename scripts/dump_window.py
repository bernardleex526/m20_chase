#!/usr/bin/env python3
"""Dump scenario samples in a time window with target truth vs estimate.

Usage: dump_window.py <scenario> <t0> <t1> [step]
"""
import json
import os
import sys

D = os.path.expanduser("~/m20_chase/evidence/scenarios")
name, t0, t1 = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
step = float(sys.argv[4]) if len(sys.argv) > 4 else 0.5

rows = [json.loads(l) for l in open(os.path.join(D, f"{name}.jsonl"))]
print(f"{'t':>6} {'dog':>16} {'tgt':>14} {'est':>14} {'rng':>6} {'cmd':>18} status")
for r in rows:
    if not (t0 <= r["t"] <= t1):
        continue
    if abs((r["t"] - t0) / step - round((r["t"] - t0) / step)) > 0.35:
        continue
    d = r.get("dog") or [0, 0, 0]
    g = r.get("tgt")
    e = r.get("est")
    rg = r.get("range")
    c = r.get("cmd") or [0, 0, 0]
    print(f"{r['t']:6.1f} ({d[0]:5.2f},{d[1]:5.2f}) "
          f"{'None' if not g else f'({g[0]:5.2f},{g[1]:5.2f})':>14} "
          f"{'None' if not e else f'({e[0]:5.2f},{e[1]:5.2f})':>14} "
          f"{'None' if rg is None else f'{rg:5.2f}':>6} "
          f"({c[0]:5.2f},{c[1]:5.2f},{c[2]:5.2f}) {r['status']}")
