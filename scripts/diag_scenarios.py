#!/usr/bin/env python3
"""Compact failure diagnosis across all scenario JSONL recordings.

For each scenario prints: min true clearance + which prop caused it, contact
names, the longest EMERGENCY_STOP stretch, and the final approach state, so the
failing criterion can be traced to a cause without reading raw logs.
"""
import json
import math
import os
import sys

D = os.path.expanduser("~/m20_chase/evidence/scenarios")
ROBOT_L, ROBOT_W = 0.62, 0.36
HALF = {"wall_long": 0.10, "pillar": 0.15, "step_low": 0.25,
        "box_small": 0.20, "ped_1": 0.25, "ped_2": 0.25, "ped_3": 0.25}
HZ = 20.0


def clearance(r):
    d = r.get("dog")
    if not d:
        return None, None
    best, who = None, None
    for name, xy in (r.get("props") or {}).items():
        dx, dy = xy[0] - d[0], xy[1] - d[1]
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            continue
        ang = math.atan2(dy, dx) - d[2]
        sup = 0.5 * (ROBOT_L * abs(math.cos(ang)) + ROBOT_W * abs(math.sin(ang)))
        c = dist - HALF.get(name, 0.25) - sup
        if best is None or c < best:
            best, who = c, name
    return best, who


names = sys.argv[1:] or sorted(
    f[:-6] for f in os.listdir(D) if f.endswith(".jsonl"))
for n in names:
    p = os.path.join(D, f"{n}.jsonl")
    if not os.path.exists(p):
        continue
    rows = [json.loads(l) for l in open(p)]
    rows = [r for r in rows if r.get("dog")]
    if not rows:
        print(f"{n}: no samples")
        continue
    mc, who = None, None
    for r in rows:
        c, w = clearance(r)
        if c is not None and (mc is None or c < mc):
            mc, who = c, w
    # longest emergency stretch
    worst = cur = 0.0
    for r in rows:
        if r["status"] == "EMERGENCY_STOP":
            cur += 1.0 / HZ
            worst = max(worst, cur)
        else:
            cur = 0.0
    contacts = rows[-1].get("contacts", 0)
    cnames = rows[-1].get("contact_names") or []
    st = {}
    for r in rows:
        st[r["status"]] = st.get(r["status"], 0) + 1
    last = rows[-1]
    d = last["dog"]
    print(f"\n=== {n} ===  n={len(rows)} dur={last['t']-rows[0]['t']:.0f}s")
    print(f"  min_clr={None if mc is None else round(mc,3)} ({who})  "
          f"contacts={contacts} {cnames}  max_ES={worst:.1f}s")
    print(f"  final dog=({d[0]:.2f},{d[1]:.2f},{math.degrees(d[2]):.0f}deg) "
          f"range={last.get('range')} cmd={[round(c,2) for c in last['cmd']]} "
          f"{last['status']}")
    print(f"  statuses={st}")
    # trajectory samples
    step = max(1, len(rows) // 10)
    for r in rows[::step]:
        dd = r["dog"]
        rg = r.get("range")
        rgs = "  n/a" if rg is None else f"{rg:5.2f}"
        print(f"    t={r['t']:6.1f} dog=({dd[0]:5.2f},{dd[1]:5.2f}) rng={rgs} "
              f"cmd=({r['cmd'][0]:5.2f},{r['cmd'][1]:5.2f},{r['cmd'][2]:5.2f}) "
              f"{r['status']}")
