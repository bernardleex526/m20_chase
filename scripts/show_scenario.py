#!/usr/bin/env python3
"""Summarise a scenario JSONL: trajectory, range, cmd, status over time."""
import json
import sys

path = sys.argv[1]
rows = [json.loads(l) for l in open(path)]
print("n=%d  t=%.1f -> %.1f" % (len(rows), rows[0]["t"], rows[-1]["t"]))
step = max(1, len(rows) // 14)
for r in rows[::step]:
    d = r.get("dog") or [0, 0, 0]
    rng = r.get("range")
    rngs = "  n/a" if rng is None else "%5.2f" % rng
    cmd = r.get("cmd") or [0, 0, 0]
    print("t=%6.1f dog=(%5.2f,%5.2f) rng=%s cmd=(%5.2f,%5.2f,%5.2f) %s"
          % (r["t"], d[0], d[1], rngs, cmd[0], cmd[1], cmd[2], r["status"]))
last = rows[-1]
print("--- last: status=%s cmd=%s range=%s est=%s"
      % (last["status"], last["cmd"], last.get("range"), last.get("est")))
st = {}
for r in rows:
    st[r["status"]] = st.get(r["status"], 0) + 1
print("statuses:", st)
