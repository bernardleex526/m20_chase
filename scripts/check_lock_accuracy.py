#!/usr/bin/env python3
"""Check whether the tracker is locked onto the RIGHT person.

Every scenario binds target_person, and the harness records both the ground-truth
range to that person (from /model_states) and the node's own target estimate
(from /rs_follow/target, sensor frame). They must agree.

A large disagreement means the node is following something else -- a wall, a
pedestrian, a prop -- while reporting TRACKING. The A1-A4 criteria do not catch
that: the robot can hold a perfect standoff from the wrong object, and A2 is
skipped entirely for scenarios where reaching the person is not the goal.

Usage: check_lock_accuracy.py [scenario ...]
"""
import json
import math
import os
import sys

D = os.path.expanduser("~/m20_chase/evidence/scenarios")
TOL = 0.35        # m; estimate vs truth disagreement that counts as a false lock

names = sys.argv[1:] or sorted(f[:-6] for f in os.listdir(D)
                               if f.endswith(".jsonl"))
bad = []
for n in names:
    p = os.path.join(D, f"{n}.jsonl")
    if not os.path.exists(p):
        continue
    rows = [json.loads(l) for l in open(p)]
    rows = [r for r in rows if r.get("dog") and r.get("est") is not None]
    if not rows:
        print(f"{n}: no samples with an estimate")
        continue
    errs, worst = [], None
    for r in rows:
        d = r["dog"]
        est = math.hypot(r["est"][0], r["est"][1])
        if r.get("range") is None:
            continue
        truth = r["range"]
        e = est - truth
        errs.append(e)
        if worst is None or abs(e) > abs(worst[0]):
            worst = (e, r["t"], est, truth, r["status"])
    if not errs:
        print(f"{n}: no comparable samples")
        continue
    frac_bad = sum(1 for e in errs if abs(e) > TOL) / len(errs)
    mean_e = sum(errs) / len(errs)
    flag = "  <== FALSE LOCK" if frac_bad > 0.10 else ""
    print(f"{n}: n={len(errs):4d}  mean_est_truth={mean_e:+6.3f} m  "
          f"|err|>{TOL}: {frac_bad*100:5.1f}%  worst {worst[0]:+6.3f} m "
          f"at t={worst[1]:.1f} (est {worst[2]:.2f} vs truth {worst[3]:.2f}, "
          f"{worst[4]}){flag}")
    if frac_bad > 0.10:
        bad.append(n)

print()
if bad:
    print(f"FALSE LOCKS in: {', '.join(bad)}")
else:
    print("no false locks: the estimate tracks the bound person everywhere")
