#!/usr/bin/env python3
"""Is S12's crossing actually solvable? Compute the geometry analytically.

Robot starts at (0,0) and must reach standoff 1.0 m from the person at (4.5, 0).
Pedestrian starts at (2.6, 1.8) and walks at 0.40 m/s toward -y.
They must cross at (2.6, 0).
"""
ROBOT_HALF_L, ROBOT_HALF_W = 0.31, 0.18
PED_R = 0.25

print("robot: 0.62 x 0.36 m, max 0.90 m/s, accel 0.80 m/s^2")
print("pedestrian: r=0.25 m, 0.40 m/s from y=+1.8 to y=-1.8 at x=2.6")
print()

# robot's time to reach x=2.6 from rest with accel 0.8 and vmax 0.9
def t_to_x(x, a=0.80, vmax=0.90):
    t_acc = vmax / a
    x_acc = 0.5 * a * t_acc ** 2
    if x <= x_acc:
        return (2 * x / a) ** 0.5
    return t_acc + (x - x_acc) / vmax

t_robot = t_to_x(2.6)
print(f"robot reaches x=2.6 at t={t_robot:.2f} s")
for y in (0.0, 0.43, 0.61):
    t_ped = (1.8 - y) / 0.40
    print(f"pedestrian reaches y={y:.2f} at t={t_ped:.2f} s")

print()
print("separation needed to be non-touching (lateral):")
need = PED_R + ROBOT_HALF_W
print(f"  pedestrian centre must stay > {need:.2f} m from the robot's centreline")
print()
print("at the robot's arrival (t=%.2f s) the pedestrian is at y=%.2f"
      % (t_robot, 1.8 - 0.40 * t_robot))
lat = 1.8 - 0.40 * t_robot
print(f"  lateral separation = {lat:.2f} m -> "
      f"{'CLEAR' if lat > need else 'CONTACT'} (need {need:.2f} m)")
print()
print("=> the robot must either arrive earlier than t=%.2f s, or dodge "
      "laterally by %.2f m" % ((1.8 - need) / 0.40, need - lat))
