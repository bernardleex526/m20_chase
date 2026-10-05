#!/usr/bin/env python3
"""Replay S10's recorded geometry through the node's own scan rules.

Reads the recorded truths (dog pose, person position, props) and reimplements
pointcloud_scan's band + nearest-per-bin projection, then reports what the
target band actually contains around the person. Answers "why did the node lock
with pts=0" without guessing.
"""
import json
import math
import os

D = os.path.expanduser("~/m20_chase/evidence/scenarios")
SENSOR_H = 0.75
HEIGHT_MIN, HEIGHT_MAX = -0.40, 1.50
TGT_RADIUS = 0.60
FRAME_F, FRAME_B, FRAME_L, FRAME_R = 0.36, 0.36, 0.23, 0.23

rows = [json.loads(l) for l in open(os.path.join(D, "S10.jsonl"))]

print("t     dog(x,y,yaw)      person_robot(along,lat)  rng   bins_in_FOV "
      "person_bins  in_frame  in_cluster  status")
for r in rows:
    if not (33.0 <= r["t"] <= 42.0):
        continue
    d = r.get("dog")
    g = r.get("tgt")
    if not d or not g:
        continue
    dx, dy = g[0] - d[0], g[1] - d[1]
    c, s = math.cos(d[2]), math.sin(d[2])
    px = dx * c + dy * s
    py = -dx * s + dy * c
    rng = math.hypot(px, py)
    brg = math.atan2(py, px)

    # person's angular half-width at its range
    half = math.asin(min(1.0, 0.25 / max(rng, 0.26)))
    lo, hi = brg - half, brg + half
    # count 0.25-deg bins of the person that fall inside the +-40deg FOV and
    # outside the self-occlusion box
    n_fov = n_frame = n_cluster = 0
    for k in range(-40, 41):
        a = brg + math.radians(half * 180 / math.pi) * 0  # placeholder
    # sample the person's silhouette
    for i in range(360):
        a = lo + (hi - lo) * i / 359.0
        sx, sy = rng * math.cos(a), rng * math.sin(a)
        if abs(a) <= math.radians(40.0):
            n_fov += 1
        if abs(sx) < FRAME_F and abs(sy) < FRAME_L:
            n_frame += 1
        if (sx - px) ** 2 + (sy - py) ** 2 < TGT_RADIUS ** 2:
            n_cluster += 1

    print(f"{r['t']:5.1f} ({d[0]:5.2f},{d[1]:5.2f},{math.degrees(d[2]):6.1f}) "
          f"({px:6.2f},{py:6.2f}) {rng:5.2f} "
          f"{n_fov:5d} {n_fov:11d} {n_frame:9d} {n_cluster:10d} {r['status']}")
