#!/usr/bin/env python3
"""Show EMERGENCY_STOP stretches and contact events for scenarios.

Usage: es_stretches.py [scenario ...]
"""
import json
import os
import sys

D = os.path.expanduser("~/m20_chase/evidence/scenarios")
HZ = 20.0
names = sys.argv[1:] or sorted(f[:-6] for f in os.listdir(D) if f.endswith(".jsonl"))

for n in names:
    p = os.path.join(D, f"{n}.jsonl")
    if not os.path.exists(p):
        continue
    rows = [json.loads(l) for l in open(p)]
    rows = [r for r in rows if r.get("dog")]
    if not rows:
        continue
    print(f"\n=== {n} ===  n={len(rows)}")
    # ES stretches
    run = []
    for r in rows:
        if r["status"] == "EMERGENCY_STOP":
            run.append(r)
        else:
            if len(run) >= 5:
                print(f"  ES  {run[0]['t']:6.1f} -> {run[-1]['t']:6.1f} s "
                      f"({len(run)/HZ:.1f} s)  dog={[round(v,2) for v in run[0]['dog']]}")
            run = []
    if len(run) >= 5:
        print(f"  ES  {run[0]['t']:6.1f} -> {run[-1]['t']:6.1f} s ({len(run)/HZ:.1f} s) [end]")
    # contact events
    ce = rows[-1].get("contact_events") or []
    for c in ce[:8]:
        print(f"  contact t={c['t']} prop={c['prop']} dog={c['dog']}")
    # recovery FSM states seen
    st = sorted(set(r.get("state", "-").split()[0] for r in rows))
    print(f"  fsm={st}")
    # final
    last = rows[-1]
    print(f"  final t={last['t']:.1f} dog={[round(v,2) for v in last['dog']]} "
          f"range={None if last.get('range') is None else round(last['range'],2)} "
          f"status={last['status']}")
