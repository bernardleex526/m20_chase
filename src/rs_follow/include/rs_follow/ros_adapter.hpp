/**
 * @file ros_adapter.hpp
 * @brief Convert ROS 2 messages to/from the framework-independent core types.
 *
 * This is the ONLY place in the package that knows both worlds. Keeping the
 * conversion in one header means:
 *   * `rs_follow_core` stays linkable from a non-ROS program,
 *   * there is exactly one place to audit when a message layout changes,
 *   * the unit tests can build the same `PointCloud` a live sensor produces
 *     without pulling in `rclcpp`.
 */

#ifndef RS_FOLLOW_ROS_ADAPTER_HPP
#define RS_FOLLOW_ROS_ADAPTER_HPP

#include <cstdint>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "rs_follow_core/types.hpp"

namespace rs_follow
{

/** @brief Nanoseconds since the epoch from a builtin_interfaces stamp. */
inline uint64_t toNanoseconds(const builtin_interfaces::msg::Time & t)
{
  return static_cast<uint64_t>(t.sec) * 1000000000ull +
         static_cast<uint64_t>(t.nanosec);
}

/** @brief True when the cloud carries a FLOAT32 field with this name. */
inline bool hasFloatField(const sensor_msgs::msg::PointCloud2 & msg, const std::string & name)
{
  for (const auto & f : msg.fields) {
    if (f.name == name && f.datatype == sensor_msgs::msg::PointField::FLOAT32) {
      return true;
    }
  }
  return false;
}

/**
 * @brief Copy a PointCloud2 into a framework-independent PointCloud.
 *
 * Returns false when the message lacks float32 x/y/z fields. The conversion is
 * a straight copy through the point iterators; it allocates once, up front.
 */
inline bool toPointCloud(const sensor_msgs::msg::PointCloud2 & msg, PointCloud & out)
{
  if (!hasFloatField(msg, "x") || !hasFloatField(msg, "y") || !hasFloatField(msg, "z")) {
    return false;
  }
  out.clear();
  out.frame_id = msg.header.frame_id;
  out.timestamp_ns = toNanoseconds(msg.header.stamp);
  out.reserve(static_cast<size_t>(msg.width) * msg.height);
  try {
    sensor_msgs::PointCloud2ConstIterator<float> it_x(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(msg, "z");
    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
      out.push_back(*it_x, *it_y, *it_z);
    }
  } catch (const std::runtime_error &) {
    return false;
  }
  return true;
}

/** @brief Copy a nav_msgs/Odometry into the core Odometry type. */
inline void toOdometry(const nav_msgs::msg::Odometry & msg, Odometry & out)
{
  const auto & p = msg.pose.pose.position;
  const auto & q = msg.pose.pose.orientation;
  out.x = p.x;
  out.y = p.y;
  out.yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                       1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  out.vx = msg.twist.twist.linear.x;
  out.vy = msg.twist.twist.linear.y;
  out.wz = msg.twist.twist.angular.z;
  out.timestamp_ns = toNanoseconds(msg.header.stamp);
  out.pose_valid = true;
  out.twist_valid = true;
}

/** @brief Fill a geometry_msgs/Twist from a core Twist. */
inline void toRosTwist(const Twist & in, geometry_msgs::msg::Twist & out)
{
  out.linear.x = in.linear_x;
  out.linear.y = in.linear_y;
  out.linear.z = 0.0;
  out.angular.x = 0.0;
  out.angular.y = 0.0;
  out.angular.z = in.angular_z;
}

/** @brief Convert a geometry_msgs/Twist into a core Twist. */
inline Twist fromRosTwist(const geometry_msgs::msg::Twist & in)
{
  return Twist{in.linear.x, in.linear.y, in.angular.z};
}

}  // namespace rs_follow

#endif  // RS_FOLLOW_ROS_ADAPTER_HPP
