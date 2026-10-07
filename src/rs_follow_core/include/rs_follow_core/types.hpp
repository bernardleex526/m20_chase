/**
 * @file types.hpp
 * @brief Framework-independent data types for the follow algorithm.
 *
 * WHY THIS FILE EXISTS
 * --------------------
 * The algorithm used to speak ROS natively: `sensor_msgs::msg::PointCloud2` came
 * in, `geometry_msgs::msg::Twist` went out, and `rclcpp::Time` was stamped on
 * every scan frame. That coupled the control law to the ROS 2 middleware, so the
 * same code could not be unit-tested without a ROS runtime, could not be linked
 * into a non-ROS upper-computer program, and could not be reused from Python
 * without a full ROS 2 installation.
 *
 * Everything in `rs_follow_core` is written against the types below instead.
 * They are plain aggregates with no dependency beyond the C++ standard library,
 * so the whole algorithm library compiles with a bare `g++ -std=c++17`.
 *
 * Conventions (identical to the ROS message conventions the node still uses):
 *   - right-handed sensor frame: x forward, y left, z up
 *   - angles in radians, 0 = straight ahead, positive = counter-clockwise (left)
 *   - distances in metres, times in seconds unless a field says `_ns`
 */

#ifndef RS_FOLLOW_CORE_TYPES_HPP
#define RS_FOLLOW_CORE_TYPES_HPP

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace rs_follow
{

/** @brief A single XYZ point (m), sensor frame. */
struct Point3
{
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

/**
 * @brief A 3D point cloud in the sensor frame.
 *
 * Struct-of-arrays: the projection loop walks x/y/z in lockstep and only ever
 * needs the three components, so this layout avoids a per-point stride
 * calculation and lets a caller fill the buffers straight from whatever it has
 * (a PCL cloud, a UDP driver, a file, or a synthetic test).
 */
struct PointCloud
{
  std::vector<float> x;
  std::vector<float> y;
  std::vector<float> z;
  std::string frame_id;
  uint64_t timestamp_ns = 0;

  std::size_t size() const {return x.size();}
  bool empty() const {return x.empty();}
  void clear()
  {
    x.clear();
    y.clear();
    z.clear();
  }
  void reserve(std::size_t n)
  {
    x.reserve(n);
    y.reserve(n);
    z.reserve(n);
  }
  void resize(std::size_t n)
  {
    x.resize(n);
    y.resize(n);
    z.resize(n);
  }
  void push_back(float px, float py, float pz)
  {
    x.push_back(px);
    y.push_back(py);
    z.push_back(pz);
  }
  Point3 at(std::size_t i) const {return Point3{x[i], y[i], z[i]};}
};

/**
 * @brief Robot pose and body velocity in an inertial frame.
 *
 * `pose_valid` and `twist_valid` are separate on purpose: a caller may have a
 * wheel odometer that reports velocity but no absolute pose, and the inertial
 * target filter only needs the pose while the obstacle-velocity estimator only
 * needs the twist.
 */
struct Odometry
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double vx = 0.0;
  double vy = 0.0;
  double wz = 0.0;
  uint64_t timestamp_ns = 0;
  bool pose_valid = false;
  bool twist_valid = false;
};

/** @brief Velocity command for a holonomic base (or a differential one, y = 0). */
struct Twist
{
  double linear_x = 0.0;
  double linear_y = 0.0;
  double angular_z = 0.0;

  /** @brief Commanded translation magnitude (m/s). */
  double linear() const {return std::hypot(linear_x, linear_y);}

  void zero()
  {
    linear_x = 0.0;
    linear_y = 0.0;
    angular_z = 0.0;
  }
};

/** @brief 2D pose, used wherever only a planar pose is meaningful. */
struct Pose2D
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

}  // namespace rs_follow

#endif  // RS_FOLLOW_CORE_TYPES_HPP
