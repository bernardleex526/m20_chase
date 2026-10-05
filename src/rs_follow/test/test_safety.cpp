/**
 * @file test_safety.cpp
 * @brief Behavioural checks for the obstacle-avoidance layer, on synthetic scans.
 *
 * The Gazebo scenarios take minutes per run and mix in the tracker, the planner
 * and the physics. This test drives FollowController directly with hand-built
 * scans so each safety property is checked in milliseconds. Every case below
 * corresponds to a real failure that was observed in simulation.
 */
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

#include "rs_follow/follow_controller.hpp"

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

struct Obs {double range; double bearing_deg; bool low_band;};

ScanFrame makeScan(const std::vector<Obs> & obs, int bins = 1440)
{
  ScanFrame s;
  s.angle_min = -M_PI;
  s.angle_increment = (2.0 * M_PI) / bins;
  s.ranges.assign(static_cast<size_t>(bins), std::numeric_limits<float>::infinity());
  s.low_ranges.assign(static_cast<size_t>(bins), std::numeric_limits<float>::infinity());
  for (const auto & o : obs) {
    const double a = o.bearing_deg * M_PI / 180.0;
    int b = static_cast<int>((a - s.angle_min) / s.angle_increment);
    if (b < 0) {b = 0;}
    if (b >= bins) {b = bins - 1;}
    // Keep the NEAREST return per bin, exactly like projectPointCloud. Plain
    // assignment let a farther point overwrite a nearer one in the same bin,
    // which silently erased the obstacles these tests are about.
    if (o.low_band) {
      if (o.range < s.low_ranges[static_cast<size_t>(b)]) {
        s.low_ranges[static_cast<size_t>(b)] = static_cast<float>(o.range);
      }
    } else {
      if (o.range < s.ranges[static_cast<size_t>(b)]) {
        s.ranges[static_cast<size_t>(b)] = static_cast<float>(o.range);
      }
    }
  }
  return s;
}

FollowConfig baseConfig()
{
  FollowConfig c;
  c.auto_frame = true;
  c.robot_length = 0.62;
  c.robot_width = 0.36;
  c.follow_dist = 1.0;
  c.target_radius = 0.6;
  c.auto_select_front = false;
  c.governor.enable = true;
  c.vfh.enable = true;
  c.vfh.auto_d_safe = true;
  return c;
}

}  // namespace

int main()
{
  std::printf("=== safety layer behaviour ===\n");

  // 1. A wall BESIDE the robot must not stop a forward drive. This is the
  //    original bug: a fixed 0.35 m omnidirectional ring froze the robot for a
  //    wall 0.30 m to the side that it would never have hit.
  {
    std::printf("\n1. a side wall must not stop a forward drive\n");
    // (a) corridor wall at 0.60 m: clearance 0.60 - 0.18 = 0.42 m, so the robot
    //     must drive at full speed. A fixed-radius omnidirectional stop ring
    //     (the original 0.35 m bug) or an over-wide lateral margin would brake
    //     here for geometry it is nowhere near hitting.
    {
      FollowController fc;
      fc.setConfig(baseConfig());
      std::vector<Obs> obs;
      for (double x = 0.2; x < 3.0; x += 0.05) {
        obs.push_back({std::hypot(x, 0.60), std::atan2(0.60, x) * 180.0 / M_PI, false});
      }
      for (double off = -8; off <= 8; off += 1.0) {
        obs.push_back({2.5, off, false});
      }
      ScanFrame s = makeScan(obs);
      fc.bindTarget(2.5, 0.0, s);
      FollowResult r = fc.update(s);
      check(r.target_valid, "target stays bound");
      check(!r.emergency_stop, "no emergency stop for a wall 0.60 m to the side");
      check(r.cmd.linear.x > 0.7, "drives at essentially full speed past it");
    }
    // (b) wall at 0.30 m: only 0.12 m of true clearance for a 0.36 m body, so
    //     slowing is CORRECT -- but it must still creep forward, not freeze.
    {
      FollowController fc;
      fc.setConfig(baseConfig());
      std::vector<Obs> obs;
      for (double x = 0.2; x < 3.0; x += 0.05) {
        obs.push_back({std::hypot(x, 0.30), std::atan2(0.30, x) * 180.0 / M_PI, false});
      }
      for (double off = -8; off <= 8; off += 1.0) {
        obs.push_back({2.5, off, false});
      }
      ScanFrame s = makeScan(obs);
      fc.bindTarget(2.5, 0.0, s);
      FollowResult r = fc.update(s);

      check(!r.emergency_stop, "no emergency stop for a wall 0.30 m to the side");
      check(r.cmd.linear.x > 0.02, "still creeps forward rather than freezing");
    }
  }

  // 2. An obstacle dead ahead must limit the speed and must never be driven
  //    through.
  {
    std::printf("\n2. obstacle dead ahead limits speed\n");
    for (double d : {2.0, 1.2, 0.8, 0.6, 0.45}) {
      FollowController fc;
      fc.setConfig(baseConfig());
      std::vector<Obs> obs{{d, 0.0, false}};
      for (double off = -25; off <= 25; off += 1.0) {
        obs.push_back({d + 3.0, off, false});
      }
      ScanFrame s = makeScan(obs);
      fc.bindTarget(d + 3.0, 0.0, s);
      FollowResult r = fc.update(s);
      char buf[200];
      std::snprintf(buf, sizeof(buf), "at %.2f m: vx=%.3f vlim=%.3f stopped=%s",
                    d, r.cmd.linear.x, r.speed_limit,
                    r.emergency_stop ? "yes" : "no");
      check(r.cmd.linear.x <= baseConfig().governor.v_cap, buf);
      if (d <= 0.45) {
        check(r.cmd.linear.x < 0.15, "very close ahead -> stopped or creeping");
      }
    }
  }

  // 3. A 0.90 m doorway must be passable: the hard stop must not fire merely
  //    because geometry sits near the body.
  {
    std::printf("\n3. 0.90 m doorway is passable\n");
    FollowController fc;
    fc.setConfig(baseConfig());
    std::vector<Obs> obs;
    for (double x = 0.6; x < 2.2; x += 0.05) {
      for (double y : {0.45, -0.45}) {
        obs.push_back({std::hypot(x, y), std::atan2(y, x) * 180.0 / M_PI, false});
      }
    }
    for (double off = -8; off <= 8; off += 1.0) {
      obs.push_back({4.0, off, false});
    }
    ScanFrame s = makeScan(obs);
    fc.bindTarget(4.0, 0.0, s);
    FollowResult r = fc.update(s);
    check(!r.emergency_stop, "doorway does not trigger the hard stop");
    check(r.cmd.linear.x > 0.15, "drives into the doorway");
  }

  // 4. Something inside the swept corridor must limit the speed.
  {
    std::printf("\n4. near-lateral obstacle limits speed\n");
    FollowController fc;
    fc.setConfig(baseConfig());
    std::vector<Obs> obs;
    for (double off = -8; off <= 8; off += 1.0) {
      obs.push_back({4.0, off, false});
    }
    obs.push_back({std::hypot(0.5, -0.42),
                   std::atan2(-0.42, 0.5) * 180.0 / M_PI, false});
    ScanFrame s = makeScan(obs);
    fc.bindTarget(4.0, 0.0, s);
    FollowResult r = fc.update(s);
    check(r.speed_limit < baseConfig().max_linear,
          "speed limited by the near-lateral obstacle");
  }

  // 5. The low band sees a step the target band cannot.
  {
    std::printf("\n5. low band detects a below-torso obstacle\n");
    FollowController fc;
    fc.setConfig(baseConfig());
    std::vector<Obs> obs;
    for (double off = -8; off <= 8; off += 1.0) {
      obs.push_back({4.0, off, false});
    }
    obs.push_back({1.0, 0.0, true});
    ScanFrame s = makeScan(obs);
    fc.bindTarget(4.0, 0.0, s);
    FollowResult r = fc.update(s);
    check(r.min_obstacle_dist < 1.5, "low-band return enters the obstacle set");
    check(r.speed_limit < baseConfig().max_linear, "and limits the speed");
  }

  // 6. The followed person is not an obstacle, in either band. The person's
  //    legs fall inside the low band, which made the robot hard-stop on its own
  //    follower at close standoff.
  {
    std::printf("\n6. the followed person is excluded from both bands\n");
    FollowController fc;
    fc.setConfig(baseConfig());
    std::vector<Obs> obs;
    for (double off = -20; off <= 20; off += 1.0) {
      obs.push_back({0.55, off, false});
      obs.push_back({0.55, off, true});
    }
    ScanFrame s = makeScan(obs);
    fc.bindTarget(0.55, 0.0, s);
    FollowResult r = fc.update(s);
    check(r.points_in_target > 0, "person clustered as the target, not an obstacle");
    check(!r.emergency_stop, "person at 0.55 m does not trigger a hard stop");
    check(r.min_obstacle_dist > 0.5 || !std::isfinite(r.min_obstacle_dist),
          "person contributes no obstacle closer than 0.5 m");
  }

  std::printf("\n=== %s (%d failure%s) ===\n",
              failures == 0 ? "ALL PASS" : "FAILURES",
              failures, failures == 1 ? "" : "s");
  return failures == 0 ? 0 : 1;
}