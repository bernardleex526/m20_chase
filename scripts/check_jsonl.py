#!/usr/bin/env python3
"""Sanity-check a recorded JSONL: movement range, scan/cloud coverage, statuses."""
import json, sys, math

for path in sys.argv[1:]:
    rows = [json.loads(l) for l in open(path) if l.strip()]
    if not rows:
        print(f"{path}: EMPTY"); continue
    dogs = [r["dog"] for r in rows if r["dog"][0] is not None]
    tgts = [r["tgt"] for r in rows if r.get("tgt")]
    scans = sum(1 for r in rows if r.get("scan"))
    clouds = sum(1 for r in rows if r.get("cloud"))
    ests = sum(1 for r in rows if r.get("est"))
    st = {}
    for r in rows:
        st[r["status"]] = st.get(r["status"], 0) + 1
    nz = sum(1 for r in rows if any(abs(c) > 1e-6 for c in r["cmd"]))
    print(f"\n=== {path} ===")
    print(f"  samples={len(rows)} duration={rows[-1]['t']-rows[0]['t']:.1f}s  scan={scans} cloud={clouds} est={ests} cmd!=0 {nz}")
    if dogs:
        xs = [d[0] for d in dogs]; ys = [d[1] for d in dogs]
        print(f"  dog x [{min(xs):.2f},{max(xs):.2f}] y [{min(ys):.2f},{max(ys):.2f}]")
    if tgts:
        xs = [t[0] for t in tgts]; ys = [t[1] for t in tgts]
        print(f"  tgt x [{min(xs):.2f},{max(xs):.2f}] y [{min(ys):.2f},{max(ys):.2f}]")
    print(f"  statuses: {st}")
    if scans:
        s0 = next(r["scan"] for r in rows if r.get("scan"))
        print(f"  scan bins={len(s0)} first={s0[0]} mid={s0[len(s0)//2]}")
    if clouds:
        c0 = next(r["cloud"] for r in rows if r.get("cloud"))
        print(f"  cloud pts={len(c0)//2} first=({c0[0]},{c0[1]})")
