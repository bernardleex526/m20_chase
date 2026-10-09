#ifndef RS_FOLLOW_BINDING_HPP
#define RS_FOLLOW_BINDING_HPP

#include <cmath>
#include <cstdint>
#include <string>
#include <geometry_msgs/msg/point_stamped.hpp>
#include "rs_follow/follow_controller.hpp"

namespace rs_follow
{

/** Validate a stamped selection before committing the controller's target.
 * Lookup resolves control_frame <- source at the supplied stamp, never latest.
 * Every failure leaves both the controller and response target untouched.
 */
template<typename Lookup>
std::string validateBindingPoint(
  const geometry_msgs::msg::PointStamped & point, const ScanFrame * scan,
  bool scan_fresh, double max_selection_age_s,
  const std::string & control_frame, const BodyFrameConfig & body,
  Lookup && lookup, FollowController & controller,
  geometry_msgs::msg::PointStamped & target)
{
  const auto & stamp = point.header.stamp;
  if (point.header.frame_id.empty() || !std::isfinite(point.point.x) ||
    !std::isfinite(point.point.y) || !std::isfinite(point.point.z) || stamp.sec < 0 ||
    stamp.nanosec >= 1000000000u || (stamp.sec == 0 && stamp.nanosec == 0))
  {
    return "BAD_POINT";
  }
  if (!scan) {return "NO_SCAN";}
  if (!scan_fresh) {return "STALE_SCAN";}
  // A selection must describe the current scan, not an old displayed frame.
  const int64_t point_ns = static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
  const int64_t age_ns = scan->stamp.nanoseconds() - point_ns;
  if (static_cast<double>(age_ns) / 1e9 > max_selection_age_s || age_ns < -100000000LL) {
    return "STALE_SCAN";
  }
  RigidTransform transform;
  if (point.header.frame_id != control_frame &&
    !lookup(point.header.frame_id, stamp, transform))
  {
    return "TF_UNAVAILABLE";
  }
  double x = point.point.x, y = point.point.y, z = point.point.z;
  transform.transform(x, y, z);
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
    insideBodyFrame(body, x, y))
  {
    return "BAD_POINT";
  }
  if (!controller.bindTarget(x, y, *scan)) {return "NO_RETURN";}
  target.header.frame_id = control_frame;
  target.header.stamp = scan->stamp;
  target.point.x = controller.targetX();
  target.point.y = controller.targetY();
  target.point.z = 0.0;
  return "OK";
}

}  // namespace rs_follow
#endif  // RS_FOLLOW_BINDING_HPP
