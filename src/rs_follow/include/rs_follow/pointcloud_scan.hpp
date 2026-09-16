/**
 * @file pointcloud_scan.hpp
 * @brief Project a 3D RoboSense point cloud into a 2D polar scan.
 *
 * The 3D cloud is sliced by a height band (z), then for each azimuth bin the
 * minimum horizontal range is kept. This produces a virtual LaserScan-like
 * representation that is independent of the LiDAR model / beam count, so the
 * follow logic below works with any RoboSense (or any XYZ) point cloud.
 */

#ifndef RS_FOLLOW_POINTCLOUD_SCAN_HPP
#define RS_FOLLOW_POINTCLOUD_SCAN_HPP

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace rs_follow
{

struct ProjectionConfig
{
  double height_min = -0.4;    // z band lower bound (m), in cloud frame
  double height_max = 1.8;     // z band upper bound (m)
  double z_offset = 0.0;       // shift applied before band check (m)
  int angle_bins = 1440;       // azimuth bins (1440 -> 0.25 deg)
  double range_min = 0.25;     // ignore returns closer than this (m)
  double range_max = 30.0;     // ignore returns farther than this (m)
  bool flip_x = false;         // remap axes if the cloud is mounted rotated
  bool flip_y = false;
};

struct ScanFrame
{
  rclcpp::Time stamp;
  std::string frame_id;
  std::vector<float> ranges;   // per bin, +inf when empty
  double angle_min = -M_PI;
  double angle_increment = 0.0;
  int valid_bins = 0;

  float rangeAt(int i) const { return ranges[static_cast<size_t>(i)]; }
  double angleAt(int i) const { return angle_min + angle_increment * i; }
};

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
 * @brief Project a PointCloud2 into a ScanFrame. Returns false on bad input.
 */
inline bool projectPointCloud(
  const sensor_msgs::msg::PointCloud2 & msg,
  const ProjectionConfig & cfg,
  ScanFrame & out)
{
  if (cfg.angle_bins <= 0) {
    return false;
  }
  if (!hasFloatField(msg, "x") || !hasFloatField(msg, "y") || !hasFloatField(msg, "z")) {
    return false;
  }

  out.stamp = rclcpp::Time(msg.header.stamp);
  out.frame_id = msg.header.frame_id;
  out.angle_min = -M_PI;
  out.angle_increment = (2.0 * M_PI) / static_cast<double>(cfg.angle_bins);
  const float inf = std::numeric_limits<float>::infinity();
  out.ranges.assign(static_cast<size_t>(cfg.angle_bins), inf);
  out.valid_bins = 0;

  const float rmin2 = static_cast<float>(cfg.range_min * cfg.range_min);
  const float rmax2 = static_cast<float>(cfg.range_max * cfg.range_max);

  try {
    sensor_msgs::PointCloud2ConstIterator<float> it_x(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(msg, "z");

    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
      float px = *it_x;
      float py = *it_y;
      float pz = *it_z;
      if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) {
        continue;
      }
      if (cfg.flip_x) {px = -px;}
      if (cfg.flip_y) {py = -py;}

      const float zz = pz - static_cast<float>(cfg.z_offset);
      if (zz < cfg.height_min || zz > cfg.height_max) {
        continue;
      }

      const float r2 = px * px + py * py;
      if (r2 < rmin2 || r2 > rmax2) {
        continue;
      }

      double az = std::atan2(static_cast<double>(py), static_cast<double>(px));
      int bin = static_cast<int>((az - out.angle_min) / out.angle_increment);
      if (bin < 0) {bin = 0;}
      if (bin >= cfg.angle_bins) {bin = cfg.angle_bins - 1;}

      const float r = std::sqrt(r2);
      if (r < out.ranges[static_cast<size_t>(bin)]) {
        if (!std::isfinite(out.ranges[static_cast<size_t>(bin)])) {
          ++out.valid_bins;
        }
        out.ranges[static_cast<size_t>(bin)] = r;
      }
    }
  } catch (const std::runtime_error &) {
    return false;
  }

  return true;
}

}  // namespace rs_follow

#endif  // RS_FOLLOW_POINTCLOUD_SCAN_HPP
