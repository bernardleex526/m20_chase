#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <thread>
#include <vector>
#include "rs_follow/follow_controller.hpp"

using namespace rs_follow;
namespace
{
int failures = 0;
void check(bool ok, const char * label)
{
  std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", label);
  if (!ok) {++failures;}
}
bool near(double a, double b) {return std::abs(a - b) < 1e-10;}
ScanFrame frame(int64_t ns, double x = 3.0)
{
  ScanFrame scan;
  scan.stamp = rclcpp::Time(ns, RCL_ROS_TIME);
  scan.angle_min = -M_PI;
  scan.angle_increment = 2.0 * M_PI / 1440;
  scan.ranges.assign(1440, std::numeric_limits<float>::infinity());
  scan.low_ranges = scan.ranges;
  if (std::isfinite(x)) {scan.ranges[720] = static_cast<float>(x);}
  return scan;
}
FollowConfig config()
{
  FollowConfig c;
  c.auto_select_front = false;
  c.enable_kalman = true;
  c.filter_in_world = false;
  c.kalman_gate = 0.0;
  c.kalman_r_ref_points = 0;
  c.lost_frames_timeout = 20;
  return c;
}
std::vector<FollowResult> playback(bool slow)
{
  FollowController controller(config());
  controller.bindTarget(3.0, 0.0, frame(1000000000LL));
  std::vector<FollowResult> results;
  for (int i = 0; i < 8; ++i) {
    if (slow) {std::this_thread::sleep_for(std::chrono::milliseconds(8));}
    auto scan = frame(1000000000LL + i * 100000000LL,
      i == 4 || i == 5 ? std::numeric_limits<double>::infinity() : 3.0 + i * 0.04);
    results.push_back(controller.update(scan));
  }
  return results;
}
}
int main()
{
  const auto fast = playback(false), slow = playback(true);
  bool same = fast.size() == slow.size();
  for (size_t i = 0; i < fast.size(); ++i) {
    same = same && near(fast[i].target_x, slow[i].target_x) &&
      near(fast[i].target_y, slow[i].target_y) &&
      near(fast[i].cmd.linear.x, slow[i].cmd.linear.x) &&
      near(fast[i].cmd.linear.y, slow[i].cmd.linear.y) &&
      near(fast[i].cmd.angular.z, slow[i].cmd.angular.z) &&
      fast[i].target_observed == slow[i].target_observed;
  }
  check(same, "identical stamped playback is invariant to wall delays");

  FollowController controller(config());
  KalmanFilter2D expected;
  expected.setState(3.0, 0.0);
  auto initial = frame(1000000000LL);
  check(controller.bindTarget(3.0, 0.0, initial), "initial real return binds");
  double x, y;
  expected.update(3.0, 0.0, 0.0, x, y);
  const auto first = controller.update(initial);
  check(first.target_observed && std::isfinite(first.target_x) && near(first.target_x, x),
    "first frame initializes safely without a fabricated time step");
  expected.update(3.2f, 0.0, 0.1, x, y);
  const auto moved = controller.update(frame(1100000000LL, 3.2));
  check(near(moved.target_x, x), "measurement prediction uses adjacent scan dt");
  const double vx = expected.getVelocityX();
  check(vx > 0.0, "moving target learns a positive velocity");
  for (int i = 1; i <= 3; ++i) {
    expected.predictOnly(0.1, x, y);
    const auto coast = controller.update(frame(1100000000LL + i * 100000000LL,
      std::numeric_limits<double>::infinity()));
    check(coast.target_valid && !coast.target_observed && near(coast.target_x, x),
      "consecutive missed frames predict exactly one incremental step");
  }
  expected.update(3.35f, 0.0, 0.1, x, y);
  const auto observed = controller.update(frame(1500000000LL, 3.35));
  check(observed.target_observed && near(observed.target_x, x),
    "observation after coasting advances only the adjacent scan interval");
  const double before = controller.targetX();
  const auto duplicate = controller.update(frame(1500000000LL, 3.4));
  const auto backward = controller.update(frame(1400000000LL, 3.4));
  check(!duplicate.target_observed && duplicate.cmd.linear.x == 0.0 &&
    !backward.target_observed && backward.cmd.linear.x == 0.0 &&
    near(controller.targetX(), before), "duplicate and backward scans pause without estimator advancement");
  expected.update(3.4f, 0.0, 0.1, x, y);
  const auto resumed = controller.update(frame(1600000000LL, 3.4));
  check(resumed.target_observed && near(resumed.target_x, x),
    "rejected timestamps do not change the adjacent accepted scan interval");
  const auto gap = controller.update(frame(2700000000LL, 3.4));
  check(!gap.target_valid && !controller.targetValid() && gap.cmd.linear.x == 0.0,
    "greater-than-one-second gap clears tracking and commands");
  auto c = config();
  c.auto_select_front = true;
  controller.setConfig(c);
  auto automatic_candidate = frame(2800000000LL, 3.4);
  automatic_candidate.ranges[719] = 3.4f;
  automatic_candidate.ranges[721] = 3.4f;
  check(!controller.update(automatic_candidate).target_valid,
    "discontinuity blocks automatic acquisition until explicit rebind");
  auto rebound = frame(2900000000LL, 3.4);
  check(controller.bindTarget(3.4, 0.0, rebound) && controller.update(rebound).target_observed,
    "explicit rebind permits tracking after a discontinuity");

  FollowController gated(config());
  gated.bindTarget(3.0, 0.0, initial);
  gated.update(initial);
  gated.update(frame(1100000000LL, 3.2));
  const double moving_x = gated.targetX();
  c = config();
  c.kalman_gate = 0.001;
  gated.setConfig(c);
  const auto rejected = gated.update(frame(1200000000LL, 3.5));
  check(rejected.points_in_target > 0 && !rejected.target_observed &&
    near(gated.targetX(), moving_x + vx * 0.1),
    "moving gated measurement advances exactly one prediction");
  return failures == 0 ? 0 : 1;
}
