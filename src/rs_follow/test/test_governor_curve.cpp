/**
 * @file test_governor_curve.cpp
 * @brief Print the governor's speed-vs-clearance curve.
 *
 * The TTC law is v*t_lat + v^2/(2 a_max) <= d - d_margin. This shows what speed
 * each clearance actually permits, so margins can be chosen from numbers rather
 * than by trial in an eight-minute simulation.
 */
#include <cstdio>

#include "rs_follow/speed_governor.hpp"

int main()
{
  rs_follow::SpeedGovernor g;
  rs_follow::GovernorConfig c;
  g.setConfig(c);
  std::printf("t_lat=%.2f a_max=%.2f d_hard=%.2f d_margin_slow=%.2f\n",
              c.t_lat, c.a_max, c.d_hard, c.d_margin_slow);
  std::printf("\n%10s %10s %10s\n", "clearance", "v_allow", "brake_dist");
  for (double d = 0.20; d <= 2.01; d += 0.10) {
    const double v = g.maxSpeed(d);
    std::printf("%10.2f %10.3f %10.3f\n", d, v, v * v / (2.0 * c.a_max));
  }
  std::printf("\n%10s %10s\n", "speed", "clearance_needed");
  for (double v = 0.2; v <= 1.01; v += 0.1) {
    const double d = v * c.t_lat + v * v / (2.0 * c.a_max) + c.d_margin_slow;
    std::printf("%10.2f %10.3f\n", v, d);
  }
  return 0;
}