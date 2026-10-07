/**
 * @file test_autoselect.cpp
 * @brief Check that auto-select picks a PERSON, not a wall.
 *
 * Reproduces two measured failures on synthetic scans:
 *   * inside a U-shaped trap the nearest thing ahead is the back wall, and the
 *     node used to lock onto it (it reported TRACKING_AUTO and held a 0.95 m
 *     standoff from the wall while the person stood 4.5 m away);
 *   * after a loss the robot spins and first sees the person clipped at the edge
 *     of its field of view, which an over-strict "touches the FOV edge" test
 *     rejected, so it never reacquired.
 */
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

#include "rs_follow_core/follow_controller.hpp"

using namespace rs_follow;

namespace
{
int failures = 0;

void check(bool ok, const std::string & what)
{
  std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok) {
    ++failures;
  }
}

/** Fill bins for a surface spanning [deg0, deg1] at range r. */
void addSurface(ScanFrame & s, double deg0, double deg1, double r)
{
  for (int i = 0; i < static_cast<int>(s.ranges.size()); ++i) {
    const double a = s.angleAt(i) * 180.0 / M_PI;
    if (a >= deg0 && a <= deg1) {
      s.ranges[static_cast<size_t>(i)] = static_cast<float>(r);
    }
  }
}

/** Fill bins for a cylinder of radius rad centred at (x, y) in sensor frame. */
void addPerson(ScanFrame & s, double x, double y, double rad)
{
  for (int i = 0; i < static_cast<int>(s.ranges.size()); ++i) {
    const double a = s.angleAt(i);
    const double ca = std::cos(a), sa = std::sin(a);
    const double b = x * ca + y * sa;
    const double c = x * x + y * y - rad * rad;
    const double disc = b * b - c;
    if (disc < 0.0) {
      continue;
    }
    const double t = b - std::sqrt(disc);
    if (t > 0.05) {
      float & cur = s.ranges[static_cast<size_t>(i)];
      if (!std::isfinite(cur) || t < cur) {
        cur = static_cast<float>(t);
      }
    }
  }
}

ScanFrame blank(int bins = 1440)
{
  ScanFrame s;
  s.angle_min = -M_PI;
  s.angle_increment = (2.0 * M_PI) / bins;
  s.ranges.assign(static_cast<size_t>(bins), std::numeric_limits<float>::infinity());
  s.low_ranges.assign(static_cast<size_t>(bins), std::numeric_limits<float>::infinity());
  return s;
}

FollowConfig cfgBase()
{
  FollowConfig c;
  c.auto_frame = true;
  c.auto_select_front = true;
  c.auto_front_fov_deg = 40.0;
  c.auto_select_max_range = 8.0;
  c.auto_max_target_width = 0.90;
  c.follow_dist = 1.0;
  return c;
}

}  // namespace

int main()
{
  std::printf("=== auto-select behaviour ===\n");

  // 1. A wall filling the field of view must NOT be selected.
  {
    std::printf("\n1. wall filling the FOV is rejected\n");
    FollowController fc;
    fc.setConfig(cfgBase());
    ScanFrame s = blank();
    addSurface(s, -60.0, 60.0, 1.5);
    FollowResult r = fc.update(s, 0.1);
    check(!r.target_valid, "no target selected from a wall");
  }

  // 1b. The robot's OWN body must never be selected. Its top surface sits in the
  //     target height band and the lidar is above it, so the chassis appears as a
  //     wide near cluster that passes the width test. Selecting it made the node
  //     report TRACKING while the cluster was empty (pts=0), losing the lock
  //     immediately -- the S10 reacquisition failure.
  {
    std::printf("\n1b. the robot's own chassis is not a target\n");
    FollowController fc;
    fc.setConfig(cfgBase());
    ScanFrame s = blank();
    // The chassis subtends a wide but finite arc, and the person stands beyond
    // it off to one side -- which is the real situation. A chassis covering the
    // WHOLE field of view would occlude the person in the polar projection
    // (nearest return per bin), so there would be nothing to select and the test
    // would be asserting an impossibility.
    addSurface(s, -40.0, 5.0, 0.34);       // chassis top, seen from the mast
    addPerson(s, 3.0 * std::cos(0.5), 3.0 * std::sin(0.5), 0.25);   // +29 deg
    FollowResult r = fc.update(s, 0.1);
    check(r.target_valid, "a target was still found");
    const double rng = std::hypot(fc.targetX(), fc.targetY());
    char buf[176];
    std::snprintf(buf, sizeof(buf),
                  "selected %.2f m (person at 2.75, chassis at 0.34)", rng);
    check(rng > 2.0, buf);
  }

  // 2. A person in the open must be selected.
  {
    std::printf("\n2. a lone person is selected\n");
    FollowController fc;
    fc.setConfig(cfgBase());
    ScanFrame s = blank();
    addPerson(s, 3.0, 0.0, 0.25);
    FollowResult r = fc.update(s, 0.1);
    check(r.target_valid, "person selected");
    const double rng = std::hypot(fc.targetX(), fc.targetY());
    char buf[128];
    std::snprintf(buf, sizeof(buf), "selected at %.2f m (person at 2.75)", rng);
    check(std::abs(rng - 2.75) < 0.6, buf);
  }

  // 3. Person entering at the edge of the FOV must still be selected. This is
  //    the reacquisition case: a spinning robot first sees them clipped.
  {
    std::printf("\n3. person clipped at the FOV edge is still selected\n");
    int ok_count = 0, n = 0;
    for (double bearing = 30.0; bearing <= 40.0; bearing += 5.0) {
      FollowController fc;
      fc.setConfig(cfgBase());
      ScanFrame s = blank();
      const double a = bearing * M_PI / 180.0;
      addPerson(s, 1.5 * std::cos(a), 1.5 * std::sin(a), 0.25);
      FollowResult r = fc.update(s, 0.1);
      ++n;
      if (r.target_valid) {
        ++ok_count;
      }
    }
    char buf[144];
    std::snprintf(buf, sizeof(buf), "selected at %d/%d bearings in 30..40 deg",
                  ok_count, n);
    check(ok_count == n, buf);
  }

  // 4. Partial wall ahead, person visible beside it -> choose the person, not
  //    the nearer wall. (A wall spanning the WHOLE field of view occludes
  //    everything behind it in the polar projection -- nearest return per bin --
  //    so the only way a person can be "beyond a wall" in a scan is around its
  //    edge, which is what this models.)
  {
    std::printf("\n4. nearer partial wall does not win over a visible person\n");
    FollowController fc;
    fc.setConfig(cfgBase());
    ScanFrame s = blank();
    addSurface(s, -40.0, -10.0, 2.2);      // wall on the left of the FOV only
    addPerson(s, 4.0 * std::cos(0.35), 4.0 * std::sin(0.35), 0.25);  // +20 deg
    FollowResult r = fc.update(s, 0.1);
    check(r.target_valid, "something selected");
    const double rng = std::hypot(fc.targetX(), fc.targetY());
    char buf[200];
    std::snprintf(buf, sizeof(buf),
                  "selected range %.2f m, bearing %.0f deg "
                  "(person at ~3.75 m / +20 deg, wall at 2.2 m / -40..-10 deg)",
                  rng, std::atan2(fc.targetY(), fc.targetX()) * 180.0 / M_PI);
    check(rng > 3.0, buf);
  }

  // 5. Exact geometry from the S10 reacquisition failure: the person reappears
  //    close and off to the side while the robot spins looking for them.
  {
    std::printf("\n5. near person off to the side is selected (S10 geometry)\n");
    int ok_count = 0, n = 0;
    for (double bearing = 0.0; bearing <= 40.0; bearing += 10.0) {
      FollowController fc;
      fc.setConfig(cfgBase());
      ScanFrame s = blank();
      const double a = bearing * M_PI / 180.0;
      addPerson(s, 1.20 * std::cos(a), 1.20 * std::sin(a), 0.25);
      FollowResult r = fc.update(s, 0.1);
      ++n;
      if (r.target_valid) {
        ++ok_count;
      }
    }
    char buf[144];
    std::snprintf(buf, sizeof(buf),
                  "selected at %d/%d bearings in 0..40 deg at 1.20 m",
                  ok_count, n);
    check(ok_count == n, buf);
  }

  std::printf("\n=== %s (%d failure%s) ===\n",
              failures == 0 ? "ALL PASS" : "FAILURES",
              failures, failures == 1 ? "" : "s");
  return failures == 0 ? 0 : 1;
}
