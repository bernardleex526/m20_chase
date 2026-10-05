#!/usr/bin/env python3
"""Find the moment of minimum clearance in a scenario and show node state there.

Usage: min_clearance_moment.py <scenario>
"""
import json
import math
import os
import sys

D = os.path.expanduser("~/m20_chase/evidence/scenarios")
ROBOT_L, ROBOT_W = 0.62, 0.36
CIRC = {"pillar": 0.15, "ped_1": 0.25, "ped_2": 0.25, "ped_3": 0.25}
BOX = {"wall_long": (1.50, 0.10), "wall_2": (1.50, 0.10), "wall_3": (1.50, 0.10),
       "wall_short": (0.70, 0.10), "wall_4": (0.70, 0.10),
       "step_low": (0.50, 0.25), "box_small": (0.20, 0.20)}

name = sys.argv[1]
rows = [json.loads(l) for l in open(os.path.join(D, f"{name}.jsonl"))]


def clearance(dog, prop, nm):
    dx, dy = prop[0] - dog[0], prop[1] - dog[1]
    yaw = dog[2]
    c, s = math.cos(yaw), math.sin(yaw)
    px = dx * c + dy * s
    py = -dx * s + dy * c
    hx, hy = 0.5 * ROBOT_L, 0.5 * ROBOT_W
    ex = max(0.0, abs(px) - hx)
    ey = max(0.0, abs(py) - hy)
    if nm in CIRC:
        return math.hypot(ex, ey) - CIRC[nm]
    if nm in BOX:
        bx, by = BOX[nm]
        # prop axis-aligned or yaw-rotated; treat as axis-aligned (all props are)
        gx = abs(dx) - hx - bx
        gy = abs(dy) - hy - by
        # separation is max over the SAT axes
        return max(gx, gy)
    return None


best = None
for r in rows:
    d = r.get("dog")
    if not d:
        continue
    for nm, pr in (r.get("props") or {}).items():
        cl = clearance(d, pr, nm)
        if cl is None:
            continue
        if best is None or cl < best[0]:
            best = (cl, r, nm)

if best is None:
    print("no props")
    sys.exit()
cl, r, nm = best
print(f"min clearance {cl:.4f} m vs {nm} at t={r['t']:.1f}")
print(f"  dog={[round(v,3) for v in r['dog']]}  prop={[round(v,3) for v in r['props'][nm]]}")
print(f"  cmd={[round(v,3) for v in r['cmd']]}  status={r['status']}")
print(f"  state={r.get('state')}")
print(f"  est={r.get('est')}  range={r.get('range')}")
print("\nsurrounding samples:")
for rr in rows:
    if abs(rr["t"] - r["t"]) <= 1.6:
        d = rr["dog"]
        pr = (rr.get("props") or {}).get(nm)
        c2 = clearance(d, pr, nm) if pr else None
        print(f"  t={rr['t']:6.1f} dog=({d[0]:5.2f},{d[1]:5.2f},"
              f"{math.degrees(d[2]):6.1f}) clr={c2 if c2 is None else round(c2,3)} "
              f"cmd=({rr['cmd'][0]:5.2f},{rr['cmd'][1]:5.2f},{rr['cmd'][2]:5.2f}) "
              f"{rr['status']:<16} {str(rr.get('state'))[:100]}")
