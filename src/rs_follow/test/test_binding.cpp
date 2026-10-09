#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include "rs_follow/binding.hpp"

using namespace rs_follow;
namespace
{
int failures = 0;
void check(bool ok, const char * label)
{
  std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", label);
  if (!ok) {++failures;}
}
ScanFrame blank()
{
  ScanFrame scan;
  scan.frame_id = "base_link";
  scan.stamp = rclcpp::Time(10, 25, RCL_ROS_TIME);
  scan.angle_min = -M_PI;
  scan.angle_increment = 2 * M_PI / 1440;
  scan.ranges.assign(1440, std::numeric_limits<float>::infinity());
  scan.low_ranges = scan.ranges;
  return scan;
}
geometry_msgs::msg::PointStamped point(double x, double y)
{
  geometry_msgs::msg::PointStamped p;
  p.header.frame_id = "base_link";
  p.header.stamp.sec = 10;
  p.header.stamp.nanosec = 25;
  p.point.x = x;
  p.point.y = y;
  return p;
}
}  // namespace

int main()
{
  FollowController controller;
  FollowConfig cfg;
  cfg.auto_select_front = false;
  cfg.enable_kalman = true;
  cfg.filter_in_world = false;
  cfg.lost_frames_timeout = 100;
  cfg.bind_radius = 0.4;
  controller.setConfig(cfg);
  const auto body = normalizeBodyFrame(cfg);
  auto scan = blank();
  scan.ranges[720] = 3.0f;
  geometry_msgs::msg::PointStamped output;
  output.header.frame_id = "sentinel";
  output.point.x = 99;
  int lookups = 0;
  bool available = false;
  auto lookup = [&](const std::string & source, const builtin_interfaces::msg::Time & stamp,
      RigidTransform & transform) {
      ++lookups;
      check(source == "sensor", "lookup receives requested source frame");
      check(stamp.sec == 10 && stamp.nanosec == 25, "lookup receives exact request stamp");
      transform.rotation = {0, -1, 0, 1, 0, 0, 0, 0, 1};
      transform.translation = {1, 2, 0.2};
      return available;
    };
  auto validate = [&](const geometry_msgs::msg::PointStamped & p, const ScanFrame * s, bool fresh) {
      return validateBindingPoint(p, s, fresh, 0.5, "base_link", body, lookup, controller, output);
    };
  auto p = point(3, 0);
  check(validate(p, nullptr, false) == "NO_SCAN", "no scan rejected");
  check(validate(p, &scan, false) == "STALE_SCAN", "stale scan rejected");
  check(validate(point(8, 0), &scan, true) == "NO_RETURN", "no target-band return rejected");
  check(!controller.targetValid() && output.header.frame_id == "sentinel" && output.point.x == 99,
    "failed initial requests do not commit target/output");
  check(validate(p, &scan, true) == "OK", "same-frame valid binding accepted");
  check(lookups == 0, "same-frame binding never requires TF");
  check(controller.targetValid() && controller.targetManual() && std::abs(controller.targetX() - 3) < 1e-8,
    "successful binding commits snapped manual target");
  check(output.header.frame_id == "base_link" && output.header.stamp.sec == 10 &&
    output.header.stamp.nanosec == 25, "binding response carries scan control-frame header");

  const auto saved = output;
  auto retained = [&]() {
      return controller.targetValid() && controller.targetManual() &&
             std::abs(controller.targetX() - 3) < 1e-8 && std::abs(controller.targetY()) < 1e-8 &&
             output.header == saved.header && output.point == saved.point;
    };
  p.header.frame_id = "sensor";
  check(validate(p, &scan, true) == "TF_UNAVAILABLE", "wrong-frame missing TF rejected");
  check(retained(), "missing TF preserves existing binding and response");
  available = true;
  // Rz(pi/2)*(-2,-2,0) + (1,2,.2) = (3,0,.2).
  p.point.x = -2;
  p.point.y = -2;
  check(validate(p, &scan, true) == "OK", "rotated translated selection accepted");
  check(retained(), "transformed selection snaps to correct control-frame target");
  check(validate(point(8, 0), &scan, true) == "NO_RETURN" && retained(),
    "NO_RETURN preserves previous manual binding");
  check(validate(point(3, 0), nullptr, false) == "NO_SCAN" && retained(),
    "NO_SCAN preserves previous manual binding");
  check(validate(point(3, 0), &scan, false) == "STALE_SCAN" && retained(),
    "STALE_SCAN preserves previous manual binding");
  p = point(3, 0);
  p.header.stamp.sec = 9;
  p.header.stamp.nanosec = 500000024;
  check(validate(p, &scan, true) == "STALE_SCAN" && retained(),
    "display selection older than 0.5s preserves previous binding");
  p.header.stamp.sec = 10;
  p.header.stamp.nanosec = 100000026;
  check(validate(p, &scan, true) == "STALE_SCAN" && retained(),
    "selection more than 0.1s in future preserves previous binding");
  p.header.stamp.sec = 9;
  p.header.stamp.nanosec = 500000025;
  check(validate(p, &scan, true) == "OK" && retained(), "exact 0.5s age accepted");
  p.header.stamp.sec = 10;
  p.header.stamp.nanosec = 100000025;
  check(validate(p, &scan, true) == "OK" && retained(), "exact 0.1s future accepted");
  p = point(3, 0);
  p.header.stamp.sec = 9;
  p.header.stamp.nanosec = 700000025;
  check(validateBindingPoint(p, &scan, true, 0.2, "base_link", body, lookup,
      controller, output) == "STALE_SCAN" && retained(),
    "configured shorter selection age rejects 0.3s old display");
  p.header.stamp.nanosec = 300000025;
  check(validateBindingPoint(p, &scan, true, 0.8, "base_link", body, lookup,
      controller, output) == "OK" && retained(),
    "configured longer selection age accepts 0.7s old display");
  p = point(3, 0);
  p.header.frame_id.clear();
  check(validate(p, &scan, true) == "BAD_POINT" && retained(), "empty frame fails transactionally");
  p = point(3, 0);
  p.header.stamp.sec = 0;
  p.header.stamp.nanosec = 0;
  check(validate(p, &scan, true) == "BAD_POINT" && retained(), "zero stamp cannot select latest TF");
  p.header.stamp.sec = -1;
  check(validate(p, &scan, true) == "BAD_POINT" && retained(), "negative stamp rejected");
  p.header.stamp.sec = 10;
  p.header.stamp.nanosec = 1000000000u;
  check(validate(p, &scan, true) == "BAD_POINT" && retained(), "malformed nanoseconds rejected");
  p = point(3, 0);
  p.point.z = std::numeric_limits<double>::quiet_NaN();
  check(validate(p, &scan, true) == "BAD_POINT" && retained(), "nonfinite z rejected transactionally");
  check(validate(point(0, 0), &scan, true) == "BAD_POINT" && retained(),
    "shared body mask rejects self-selection");
  auto invalid_transform = [](const std::string &, const builtin_interfaces::msg::Time &,
      RigidTransform & transform) {
      transform.translation[0] = std::numeric_limits<double>::infinity();
      return true;
    };
  p = point(3, 0);
  p.header.frame_id = "sensor";
  check(validateBindingPoint(p, &scan, true, 0.5, "base_link", body, invalid_transform,
      controller, output) == "BAD_POINT" && retained(),
    "nonfinite transformed point preserves previous binding");
  auto low_only = blank();
  low_only.low_ranges[720] = 3;
  check(validate(point(3, 0), &low_only, true) == "NO_RETURN" && retained(),
    "low-band obstacle is not eligible for target binding");
  auto observed = controller.update(scan);
  check(observed.target_valid && observed.target_observed, "real return is a new observation");
  auto empty = blank();
  for (int i = 0; i < 3; ++i) {
    empty.stamp = rclcpp::Time(scan.stamp.nanoseconds() + (i + 1) * 100000000LL,
      RCL_ROS_TIME);
    const auto predicted = controller.update(empty);
    check(predicted.target_valid && predicted.target_manual && !predicted.target_observed,
      "prediction retains binding but is not a new observation");
  }
  return failures ? 1 : 0;
}
