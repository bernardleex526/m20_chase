/**
 * @file pointcloud_scan.hpp
 * @brief Project a 3D point cloud into 2D polar scans.
 *
 * The 3D cloud is sliced by one or more height bands (z), then for each
 * azimuth bin the minimum horizontal range is kept. This produces a virtual
 * LaserScan-like representation that is independent of the LiDAR model / beam
 * count, so the follow logic below works with any (or any XYZ) point cloud.
 *
 * TWO BANDS are supported:
 *   - the TARGET band (height_min/height_max): person torso, used for target
 *     acquisition and tracking. It is what the original code used.
 *   - the LOW band (low_height_min/low_height_max): near the floor, used for
 *     OBSTACLES ONLY. A quadruped's typical failure is a step, curb or small
 *     object below the target band, which the single-band version could not
 *     see at all. Low-band returns are never eligible as a follow target.
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
  // --- target band (person torso) ---
  double height_min = -0.4;    // z band lower bound (m), in cloud frame
  double height_max = 1.8;     // z band upper bound (m)
  // --- low band (near-floor obstacles) ---
  bool enable_low_band = false;
  double low_height_min = -0.9;
  double low_height_max = -0.45;

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
  std::vector<float> ranges;   // per bin, +inf when empty (TARGET band)
  std::vector<float> low_ranges;  // per bin, +inf when empty (LOW band)
  double angle_min = -M_PI;
  double angle_increment = 0.0;
  int valid_bins = 0;
  int valid_low_bins = 0;

  float rangeAt(int i) const {return ranges[static_cast<size_t>(i)];}
  float lowRangeAt(int i) const
  {
    return low_ranges.empty() ? std::numeric_limits<float>::infinity()
                              : low_ranges[static_cast<size_t>(i)];
  }
  double angleAt(int i) const {return angle_min + angle_increment * i;}

  /** Combined obstacle range: the nearer of the two bands (either may be inf). */
  float obstacleAt(int i) const
  {
    const float a = ranges[static_cast<size_t>(i)];
    const float b = lowRangeAt(i);
    return (b < a) ? b : a;
  }
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
  out.low_ranges.assign(static_cast<size_t>(cfg.angle_bins), inf);
  out.valid_bins = 0;
  out.valid_low_bins = 0;

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
      const bool in_target_band = (zz >= cfg.height_min && zz <= cfg.height_max);
      const bool in_low_band = cfg.enable_low_band &&
        (zz >= cfg.low_height_min && zz <= cfg.low_height_max);
      if (!in_target_band && !in_low_band) {
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
      const size_t bi = static_cast<size_t>(bin);
      const float r = std::sqrt(r2);

      if (in_target_band && r < out.ranges[bi]) {
        if (!std::isfinite(out.ranges[bi])) {
          ++out.valid_bins;
        }
        out.ranges[bi] = r;
      }
      if (in_low_band && r < out.low_ranges[bi]) {
        if (!std::isfinite(out.low_ranges[bi])) {
          ++out.valid_low_bins;
        }
        out.low_ranges[bi] = r;
      }
    }
  } catch (const std::runtime_error &) {
    return false;
  }

  return true;
}

}  // namespace rs_follow

#endif  // RS_FOLLOW_POINTCLOUD_SCAN_HPP