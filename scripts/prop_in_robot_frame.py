#!/usr/bin/env python3
"""Show a prop's position in the ROBOT frame over time, to check crossing logic.

Usage: prop_in_robot_frame.py <scenario> <prop> <t0> <t1>
"""
import json
import math
import os
import sys

D = os.path.expanduser("~/m20_chase/evidence/scenarios")
name, prop = sys.argv[1], sys.argv[2]
t0 = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
t1 = float(sys.argv[4]) if len(sys.argv) > 4 else 1e9

rows = [json.loads(l) for l in open(os.path.join(D, f"{name}.jsonl"))]
print(f"{'t':>6} {'robot':>14} {'prop_world':>14} {'prop_robot':>16} "
      f"{'along':>7} {'lat':>7} {'cmd':>18} status")
prev = None
for r in rows:
    if not (t0 <= r["t"] <= t1):
        continue
    d = r.get("dog")
    p = (r.get("props") or {}).get(prop)
    if not d or not p:
        continue
    dx, dy = p[0] - d[0], p[1] - d[1]
    yaw = d[2]
    c, s = math.cos(yaw), math.sin(yaw)
    px = dx * c + dy * s          # along the robot's x axis
    py = -dx * s + dy * c         # along the robot's y axis
    vx = vy = 0.0
    if prev:
        dt = r["t"] - prev[0]
        if dt > 1e-3:
            vx = (px - prev[1]) / dt
            vy = (py - prev[2]) / dt
    prev = (r["t"], px, py)
    cmd = r.get("cmd") or [0, 0, 0]
    print(f"{r['t']:6.1f} ({d[0]:5.2f},{d[1]:5.2f}) ({p[0]:5.2f},{p[1]:5.2f}) "
          f"({px:7.2f},{py:6.2f}) {px:7.2f} {py:7.2f} "
          f"({cmd[0]:5.2f},{cmd[1]:5.2f},{cmd[2]:5.2f}) {r['status']} "
          f"v=({vx:5.2f},{vy:5.2f})")
