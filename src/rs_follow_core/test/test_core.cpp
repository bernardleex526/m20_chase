/**
 * @file test_core.cpp
 * @brief Standalone tests for rs_follow_core, with NO ROS on the include path.
 *
 * Two things are being verified here:
 *
 *   1. THE DECOUPLING IS REAL. This file includes only rs_follow_core headers
 *      and the C++ standard library. If any ROS type had leaked into the
 *      algorithm, this translation unit would not compile.
 *
 *   2. THE NEW MODULES BEHAVE. Multi-target identity (a passer-by must not
 *      steal the lock; a brief occlusion must not lose it) and dynamic obstacle
 *      prediction (a pedestrian crossing perpendicular to the robot's path has a
 *      near-zero range rate, so a range-rate test cannot see them -- the
 *      predictor must).
 *
 * The safety behaviour of the controller itself is covered by rs_follow's own
 * test_safety / test_autoselect, which now run against this same core library.
 */

#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

#include "rs_follow_core/lidar_follower.hpp"
#include "rs_follow_core/multi_target_tracker.hpp"
#include "rs_follow_core/perf_monitor.hpp"

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

/** @brief A synthetic person: an arc of returns at `range`, centred on `bearing`. */
void addPerson(ScanFrame & s, double range, double bearing_deg,
               double width_m = 0.5, bool low_band = false)
{
  const int bins = static_cast<int>(s.ranges.size());
  // angular half-width of a `width_m` wide object at `range`
  const double half = std::asin(std::min(0.99, (width_m / 2.0) / range));
  const double b = bearing_deg * M_PI / 180.0;
  const double a0 = b - half;
  const double a1 = b + half;
  for (int i = 0; i < bins; ++i) {
    const double a = s.angleAt(i);
    if (a < a0 || a > a1) {
      continue;
    }
    const float r = static_cast<float>(range);
    if (low_band) {
      if (r < s.low_ranges[static_cast<size_t>(i)]) {
        s.low_ranges[static_cast<size_t>(i)] = r;
      }
    } else {
      if (r < s.ranges[static_cast<size_t>(i)]) {
        s.ranges[static_cast<size_t>(i)] = r;
      }
    }
  }
}

/** @brief A synthetic wall: a broad arc, far wider than a person. */
void addWall(ScanFrame & s, double range, double bearing_deg, double half_deg)
{
  const int bins = static_cast<int>(s.ranges.size());
  const double b = bearing_deg * M_PI / 180.0;
  const double h = half_deg * M_PI / 180.0;
  for (int i = 0; i < bins; ++i) {
    const double a = s.angleAt(i);
    if (std::abs(wrapPi(a - b)) > h) {
      continue;
    }
    const float r = static_cast<float>(range);
    if (r < s.ranges[static_cast<size_t>(i)]) {
      s.ranges[static_cast<size_t>(i)] = r;
    }
  }
}

ScanFrame emptyScan(int bins = 1440)
{
  ScanFrame s;
  s.angle_min = -M_PI;
  s.angle_increment = (2.0 * M_PI) / bins;
  s.ranges.assign(static_cast<size_t>(bins), std::numeric_limits<float>::infinity());
  s.low_ranges.assign(static_cast<size_t>(bins), std::numeric_limits<float>::infinity());
  return s;
}

MultiTargetConfig mtConfig()
{
  MultiTargetConfig c;
  c.enable = true;
  c.select = MultiTargetConfig::Select::kIdLock;
  return c;
}

}  // namespace

int main()
{
  std::printf("=== rs_follow_core: no-ROS core tests ===\n");

  // -------------------------------------------------------------------------
  // 1. Projection works on the plain PointCloud type.
  // -------------------------------------------------------------------------
  {
    std::printf("\n1. projection from a plain PointCloud\n");
    PointCloud cloud;
    cloud.frame_id = "rslidar";
    cloud.timestamp_ns = 123456789ull;
    // a person at 3 m, straight ahead, spanning the target band
    for (double y = -0.25; y <= 0.25; y += 0.01) {
      cloud.push_back(3.0f, static_cast<float>(y), 0.5f);
    }
    // ground, which must be filtered out by the band
    for (double x = 1.0; x < 5.0; x += 0.1) {
      cloud.push_back(static_cast<float>(x), 0.0f, -0.95f);
    }
    ProjectionConfig pc;
    pc.height_min = -0.40;
    pc.height_max = 1.80;
    pc.angle_bins = 1440;
    ScanFrame scan;
    const bool ok = projectPointCloud(cloud, pc, scan);
    check(ok, "projectPointCloud accepts a plain PointCloud");
    check(scan.stamp_ns == 123456789ull, "timestamp carried through");
    check(scan.frame_id == "rslidar", "frame_id carried through");
    check(scan.valid_bins > 0, "the person produced target-band bins");
    // the ground is below height_min, so no bin should read ~1.0 m
    bool ground_leaked = false;
    for (int i = 0; i < static_cast<int>(scan.ranges.size()); ++i) {
      const float r = scan.ranges[static_cast<size_t>(i)];
      if (std::isfinite(r) && r < 2.0f) {
        ground_leaked = true;
      }
    }
    check(!ground_leaked, "ground returns outside the band are excluded");
  }

  // -------------------------------------------------------------------------
  // 2. A malformed cloud is rejected, an empty one is accepted.
  // -------------------------------------------------------------------------
  {
    std::printf("\n2. malformed vs empty input\n");
    PointCloud bad;
    bad.x = {1.0f, 2.0f};
    bad.y = {1.0f};              // not parallel
    ProjectionConfig pc;
    ScanFrame scan;
    check(!projectPointCloud(bad, pc, scan), "non-parallel component arrays rejected");

    PointCloud empty;
    check(projectPointCloud(empty, pc, scan), "an empty cloud is a valid empty frame");
    check(scan.valid_bins == 0, "and leaves every bin empty");
  }

  // -------------------------------------------------------------------------
  // 3. Multi-target: a passer-by must not steal the lock.
  // -------------------------------------------------------------------------
  {
    std::printf("\n3. multi-target identity: a passer-by does not steal the lock\n");
    MultiTargetTracker tr;
    MultiTargetConfig c;
    c.enable = true;
    c.select = MultiTargetConfig::Select::kIdLock;
    c.assoc_gate = 0.7;
    c.max_range = 8.0;
    tr.setConfig(c);

    // Frame 1: one person at 3 m ahead.
    ScanFrame s1 = emptyScan();
    addPerson(s1, 3.0, 0.0);
    tr.update(s1, 0.1);
    const int idx1 = tr.select(M_PI, 8.0);
    check(idx1 >= 0, "the person is tracked");
    const int locked = idx1 >= 0 ? tr.tracks()[static_cast<size_t>(idx1)].id : -1;
    tr.lockId(locked);
    check(locked >= 0, "and an id was assigned");

    // Frame 2: a second person appears much CLOSER but off to the side.
    ScanFrame s2 = emptyScan();
    addPerson(s2, 3.0, 0.0);       // our person, unmoved
    addPerson(s2, 1.2, 35.0);      // a passer-by, nearer
    tr.update(s2, 0.1);
    const int idx2 = tr.select(M_PI, 8.0);
    check(tr.tracks().size() >= 2, "both people are tracked as separate ids");
    check(idx2 >= 0 && tr.tracks()[static_cast<size_t>(idx2)].id == locked,
          "selection keeps the LOCKED id, not the nearest person");

    // Frame 3: the passer-by leaves; the lock must survive untouched.
    ScanFrame s3 = emptyScan();
    addPerson(s3, 3.0, 0.0);
    tr.update(s3, 0.1);
    const int idx3 = tr.select(M_PI, 8.0);
    check(idx3 >= 0 && tr.tracks()[static_cast<size_t>(idx3)].id == locked,
          "the lock still points at the same person");
  }

  // -------------------------------------------------------------------------
  // 4. Multi-target: a brief occlusion must not lose the id.
  // -------------------------------------------------------------------------
  {
    std::printf("\n4. multi-target identity survives a brief occlusion\n");
    MultiTargetTracker tr;
    MultiTargetConfig c;
    c.enable = true;
    c.select = MultiTargetConfig::Select::kIdLock;
    c.max_missed = 15;
    tr.setConfig(c);

    ScanFrame s1 = emptyScan();
    addPerson(s1, 3.0, 0.0);
    tr.update(s1, 0.1);
    const int idx = tr.select(M_PI, 8.0);
    const int locked = tr.tracks()[static_cast<size_t>(idx)].id;
    tr.lockId(locked);

    // 5 frames with the person completely hidden.
    for (int i = 0; i < 5; ++i) {
      tr.update(emptyScan(), 0.1);
    }
    check(!tr.tracks().empty(), "the track is PREDICTED through the occlusion, not deleted");
    const TargetTrack * t = tr.findById(locked);
    check(t != nullptr, "the same id is still present");
    check(t && t->confidence < 1.0, "its confidence decayed while unmeasured");

    // Person reappears slightly moved: must re-associate to the SAME id.
    ScanFrame s2 = emptyScan();
    addPerson(s2, 3.2, 4.0);
    tr.update(s2, 0.1);
    const int idx2 = tr.select(M_PI, 8.0);
    check(idx2 >= 0 && tr.tracks()[static_cast<size_t>(idx2)].id == locked,
          "re-associates to the same id after reappearing");
  }

  // -------------------------------------------------------------------------
  // 5. Multi-target: a wall is not a person.
  // -------------------------------------------------------------------------
  {
    std::printf("\n5. multi-target: a wall is rejected, a person is not\n");
    MultiTargetTracker tr;
    MultiTargetConfig c;
    c.enable = true;
    c.max_range = 8.0;
    c.max_width = 0.90;
    tr.setConfig(c);

    ScanFrame wall = emptyScan();
    addWall(wall, 2.5, 0.0, 40.0);      // spans the whole front FOV
    tr.update(wall, 0.1);
    check(tr.tracks().empty(), "a 40-deg-wide surface is not tracked as a person");

    tr.reset();
    ScanFrame person = emptyScan();
    addPerson(person, 2.5, 0.0, 0.5);
    tr.update(person, 0.1);
    check(!tr.tracks().empty(), "a 0.5 m wide person IS tracked");
  }

  // -------------------------------------------------------------------------
  // 6. Dynamic obstacles: a crossing pedestrian is seen by the predictor.
  //
  //    This is the case the evaluation called out: the pedestrian's RANGE stays
  //    nearly constant while their BEARING sweeps, so a range-rate test reads
  //    zero. The predictor must still flag the path as blocked.
  // -------------------------------------------------------------------------
  {
    std::printf("\n6. dynamic obstacle: crossing pedestrian is predicted\n");
    DynamicObstacleTracker dyn;
    DynamicObstacleConfig dc;
    dc.enable = true;
    dc.max_range = 6.0;
    dc.assoc_gate = 0.8;
    dc.vel_alpha = 0.5;
    dyn.setConfig(dc);

    // The pedestrian starts 4 m ahead and walks in -y (left to right) at
    // 0.4 m/s, crossing the robot's path 4 m away. The robot drives +x at
    // 0.9 m/s. In the sensor frame the pedestrian's x stays ~4.0 m while y
    // sweeps -- a range-rate test sees almost nothing.
    //
    // Enough frames are run for the pedestrian to come within the predictor's
    // 1.5 s lookahead (1.35 m of travel at 0.9 m/s); before that there is
    // genuinely no crossing to report.
    double py = 1.2;
    double px = 4.0;
    const double pvy = -0.4;
    CrossingResult cr;
    for (int i = 0; i < 30; ++i) {
      ScanFrame s = emptyScan();
      const double bearing = std::atan2(py, px) * 180.0 / M_PI;
      const double range = std::hypot(px, py);
      addPerson(s, range, bearing, 0.5);
      // the robot advances, so the pedestrian's relative x shrinks
      dyn.update(s, 0.1, 0.9, 0.0, 0.0);
      px -= 0.9 * 0.1;
      py += pvy * 0.1;
      cr = dyn.crossingCheck(0.9, 0.0, 0.0, 0.0, 0.0);
    }

    check(!dyn.tracks().empty(), "the pedestrian is tracked");
    const ObstacleTrack * ped = dyn.nearestMoving(0.15);
    check(ped != nullptr, "and recognised as MOVING (range rate alone would read ~0)");
    if (ped) {
      char buf[200];
      std::snprintf(buf, sizeof(buf),
                    "estimated world velocity (%.2f, %.2f) m/s vs true (0.00, %.2f)",
                    ped->vx, ped->vy, pvy);
      // The pedestrian's own motion is -y; the robot's +x drive must have been
      // fully removed, so vx stays near zero.
      check(ped->vy < -0.25 && std::abs(ped->vx) < 0.20, buf);
    }

    check(cr.crossing, "the crossing is detected against the intended path");
    check(dyn.maxClosingRate() >= 0.0, "a closing rate is reported");
  }

  // -------------------------------------------------------------------------
  // 7. Dynamic obstacles: a static wall must read ZERO velocity.
  //
  //    The robot's own motion has to be removed, or every wall looks like it is
  //    rushing at the robot and the governor brakes constantly.
  // -------------------------------------------------------------------------
  {
    std::printf("\n7. dynamic obstacle: a static wall reads zero velocity\n");
    DynamicObstacleTracker dyn;
    DynamicObstacleConfig dc;
    dc.enable = true;
    dc.max_range = 6.0;
    dc.vel_alpha = 0.5;
    dyn.setConfig(dc);

    // A wall at a fixed sensor-frame range while the robot "moves" is not
    // physical; the honest test is a wall whose range SHRINKS exactly as the
    // robot approaches, which is what removing the robot's own velocity undoes.
    double px = 4.0;
    for (int i = 0; i < 10; ++i) {
      ScanFrame s = emptyScan();
      addWall(s, px, 0.0, 30.0);
      dyn.update(s, 0.1, 0.9, 0.0, 0.0);   // robot closing at 0.9 m/s
      px -= 0.9 * 0.1;                     // ... so the range shrinks by exactly that
    }
    const ObstacleTrack * wall = nullptr;
    for (const auto & t : dyn.tracks()) {
      if (t.points > 0) {
        wall = &t;
        break;
      }
    }
    check(wall != nullptr, "the wall is tracked");
    if (wall) {
      char buf[200];
      std::snprintf(buf, sizeof(buf),
                    "wall world velocity (%.2f, %.2f) m/s -- should be ~0 once robot motion is removed",
                    wall->vx, wall->vy);
      check(wall->speed() < 0.35, buf);
      check(!wall->moving(0.15), "and is not classified as a moving obstacle");
    }
  }

  // -------------------------------------------------------------------------
  // 8. The perf monitor actually measures and reports.
  // -------------------------------------------------------------------------
  {
    std::printf("\n8. perf monitor\n");
    PerfMonitor perf;
    perf.enable = true;
    for (int i = 0; i < 10; ++i) {
      PerfScope s(perf, Stage::kProjection);
      volatile double x = 0.0;
      for (int k = 0; k < 1000; ++k) {
        x += std::sqrt(static_cast<double>(k));
      }
      (void)x;
      perf.tick();
    }
    check(perf.ticks() == 10, "ticks are counted");
    check(perf.stats(Stage::kProjection).count == 10, "the stage recorded 10 samples");
    check(perf.stats(Stage::kProjection).mean_ms() > 0.0, "a non-zero mean is reported");
    const std::string rep = perf.report();
    check(rep.find("projection") != std::string::npos, "the report names the stage");
    check(rep.find("rss=") != std::string::npos, "the report includes memory");
  }

  // -------------------------------------------------------------------------
  // 9. End-to-end through the facade, with no ROS anywhere.
  // -------------------------------------------------------------------------
  {
    std::printf("\n9. LidarFollower end-to-end (no ROS)\n");
    FollowerConfig cfg;
    cfg.projection.angle_bins = 1440;
    cfg.projection.height_min = -0.40;
    cfg.projection.height_max = 1.80;
    cfg.follow.auto_select_front = false;
    cfg.follow.governor.enable = true;
    cfg.follow.vfh.enable = true;
    cfg.multi_target.enable = false;      // exercise the classic path first
    cfg.dynamic_obstacle.enable = true;
    cfg.active = true;
    cfg.control_rate_hz = 50.0;

    LidarFollower follower;
    follower.setConfig(cfg);

    PointCloud cloud;
    for (double y = -0.25; y <= 0.25; y += 0.02) {
      cloud.push_back(3.0f, static_cast<float>(y), 0.5f);
    }
    check(follower.setPointCloud(cloud), "cloud accepted by the facade");

    Odometry odom;
    odom.pose_valid = true;
    odom.twist_valid = true;
    follower.setOdometry(odom);

    // bind to the person at 3 m and enable
    check(follower.bindTarget(3.0, 0.0), "target bound");
    follower.setEnabled(true);

    Twist cmd;
    for (int i = 0; i < 20; ++i) {
      cmd = follower.update(0.02);
    }
    check(cmd.linear_x > 0.0, "the robot drives toward the person");
    check(std::abs(cmd.linear_x) <= cfg.follow.max_linear + 1e-9, "within the speed limit");
    check(follower.status().scan_fresh, "the scan is reported fresh");
    check(follower.status().statusText() == "TRACKING_MANUAL", "status text is sane");
    char buf[200];
    std::snprintf(buf, sizeof(buf), "cmd=(%.3f, %.3f, %.3f) status=%s tracks=%d",
                  cmd.linear_x, cmd.linear_y, cmd.angular_z,
                  follower.status().statusText().c_str(),
                  follower.status().obstacle_track_count);
    check(true, buf);

    // Disabling must stop the robot. The output smoother ramps down at the
    // configured deceleration rather than stepping to zero (only an EMERGENCY
    // stop bypasses it), so the command decays over a few control ticks.
    follower.setEnabled(false);
    for (int i = 0; i < 60; ++i) {
      cmd = follower.update(0.02);
    }
    check(cmd.linear() < 1e-6, "disabled -> command decays to zero");
    check(follower.status().statusText() == "DISABLED", "and reports DISABLED");
  }

  // -------------------------------------------------------------------------
  // 10. A stale scan must zero the command (sensor watchdog).
  // -------------------------------------------------------------------------
  {
    std::printf("\n10. stale scan watchdog\n");
    FollowerConfig cfg;
    cfg.follow.auto_select_front = false;
    cfg.multi_target.enable = false;
    cfg.cmd_timeout = 0.3;
    LidarFollower follower;
    follower.setConfig(cfg);

    PointCloud cloud;
    for (double y = -0.25; y <= 0.25; y += 0.02) {
      cloud.push_back(3.0f, static_cast<float>(y), 0.5f);
    }
    follower.setPointCloud(cloud);
    follower.bindTarget(3.0, 0.0);
    follower.setEnabled(true);
    for (int i = 0; i < 5; ++i) {
      follower.update(0.02);
    }
    check(follower.status().scan_fresh, "fresh while scans arrive");
    // no more scans: age past cmd_timeout. The command decays through the
    // smoother rather than stepping to zero, so allow the ramp to finish.
    Twist cmd;
    for (int i = 0; i < 60; ++i) {
      cmd = follower.update(0.02);
    }
    check(!follower.status().scan_fresh, "scan reported stale after cmd_timeout");
    check(cmd.linear() < 1e-6, "stale scan -> command decays to zero");
  }

  std::printf("\n=== %s (%d failure%s) ===\n",
              failures == 0 ? "ALL PASS" : "FAILURES",
              failures, failures == 1 ? "" : "s");
  return failures == 0 ? 0 : 1;
}
