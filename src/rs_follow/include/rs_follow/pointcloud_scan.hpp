/**
 * @file pointcloud_scan.hpp
 * @brief Project stamped XYZ clouds into control-frame target and obstacle scans.
 */
#ifndef RS_FOLLOW_POINTCLOUD_SCAN_HPP
#define RS_FOLLOW_POINTCLOUD_SCAN_HPP

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace rs_follow
{

struct BodyFrameConfig
{
  double frame_front = 0.25;
  double frame_back = 0.45;
  double frame_left = 0.25;
  double frame_right = 0.25;
  double robot_length = 0.62;
  double robot_width = 0.36;
  bool auto_frame = true;
  double self_occlusion_margin = 0.05;
};

inline BodyFrameConfig normalizeBodyFrame(const BodyFrameConfig & cfg)
{
  BodyFrameConfig result = cfg;
  if (result.auto_frame) {
    result.frame_front = result.frame_back =
      0.5 * result.robot_length + result.self_occlusion_margin;
    result.frame_left = result.frame_right =
      0.5 * result.robot_width + result.self_occlusion_margin;
  }
  return result;
}

inline bool insideBodyFrame(const BodyFrameConfig & body, double x, double y)
{
  return x > -body.frame_back && x < body.frame_front &&
         y > -body.frame_right && y < body.frame_left;
}

struct RigidTransform
{
  std::array<double, 9> rotation{{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}};
  std::array<double, 3> translation{{0.0, 0.0, 0.0}};

  void transform(double & x, double & y, double & z) const
  {
    const double nx = rotation[0] * x + rotation[1] * y + rotation[2] * z + translation[0];
    const double ny = rotation[3] * x + rotation[4] * y + rotation[5] * z + translation[1];
    z = rotation[6] * x + rotation[7] * y + rotation[8] * z + translation[2];
    x = nx;
    y = ny;
  }
};

struct PointXYZ {float x; float y; float z;};

struct ProjectionConfig
{
  // All bounds and bands are in the control frame, not the incoming cloud frame.
  double height_min = 0.35;
  double height_max = 2.55;
  bool enable_low_band = false;
  double low_height_min = 0.10;
  double low_height_max = 0.30;
  int angle_bins = 1440;
  double range_min = 0.25;
  double range_max = 30.0;
  BodyFrameConfig body = normalizeBodyFrame(BodyFrameConfig{});
};

struct ScanFrame
{
  rclcpp::Time stamp;
  std::string frame_id;
  std::vector<float> ranges;
  std::vector<float> low_ranges;
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
  float obstacleAt(int i) const
  {
    const float a = ranges[static_cast<size_t>(i)];
    const float b = lowRangeAt(i);
    return (b < a) ? b : a;
  }
};

namespace projection_detail
{
inline bool floatOffset(
  const sensor_msgs::msg::PointCloud2 & msg, const char * name, uint32_t & offset)
{
  bool found = false;
  for (const auto & field : msg.fields) {
    if (field.name != name) {continue;}
    if (found || field.datatype != sensor_msgs::msg::PointField::FLOAT32 ||
      field.count != 1 || field.offset > msg.point_step || msg.point_step - field.offset < 4)
    {
      return false;
    }
    found = true;
    offset = field.offset;
  }
  return found;
}

inline float readFloat(const uint8_t * data, bool big_endian)
{
  const uint32_t bits = big_endian ?
    (uint32_t(data[0]) << 24 | uint32_t(data[1]) << 16 | uint32_t(data[2]) << 8 | data[3]) :
    (uint32_t(data[3]) << 24 | uint32_t(data[2]) << 16 | uint32_t(data[1]) << 8 | data[0]);
  float value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

// Fixed-capacity open addressing avoids per-point allocations. Created only
// when visualization is requested; the first point in each voxel wins.
struct DisplayVoxels
{
  struct Slot {int64_t x = 0; int64_t y = 0; int64_t z = 0; bool occupied = false;};
  std::array<Slot, 16384> slots{};

  bool insert(double x, double y, double z)
  {
    const double fx = std::floor(x / 0.1);
    const double fy = std::floor(y / 0.1);
    const double fz = std::floor(z / 0.1);
    const double bound = 9223372036854775808.0;
    if (fx < -bound || fx >= bound || fy < -bound || fy >= bound ||
      fz < -bound || fz >= bound) {return false;}
    const int64_t ix = static_cast<int64_t>(fx);
    const int64_t iy = static_cast<int64_t>(fy);
    const int64_t iz = static_cast<int64_t>(fz);
    uint64_t hash = (static_cast<uint64_t>(ix) * 1099511628211ULL) ^
      (static_cast<uint64_t>(iy) * 14029467366897019727ULL) ^
      (static_cast<uint64_t>(iz) * 1609587929392839161ULL);
    size_t index = static_cast<size_t>(hash & (slots.size() - 1));
    for (;;) {
      auto & slot = slots[index];
      if (!slot.occupied) {
        slot = Slot{ix, iy, iz, true};
        return true;
      }
      if (slot.x == ix && slot.y == iy && slot.z == iz) {return false;}
      index = (index + 1) & (slots.size() - 1);
    }
  }
};
}  // namespace projection_detail

/** Returns false for invalid layout/configuration, without altering outputs.
 * Empty well-formed clouds succeed with empty scans. NaN/Inf points are skipped.
 * Self returns are removed before either per-bin minimum is selected.
 */
inline bool projectPointCloud(
  const sensor_msgs::msg::PointCloud2 & msg, const ProjectionConfig & cfg,
  ScanFrame & out, const RigidTransform & tf = RigidTransform{},
  const std::string & control_frame = "", std::vector<PointXYZ> * display = nullptr)
{
  uint32_t ox = 0, oy = 0, oz = 0;
  if (msg.header.stamp.sec < 0 || msg.header.stamp.nanosec >= 1000000000U) {return false;}
  if (cfg.angle_bins <= 0 || !std::isfinite(cfg.range_min) ||
    !std::isfinite(cfg.range_max) || cfg.range_min < 0 || cfg.range_max < cfg.range_min ||
    !std::isfinite(cfg.height_min) || !std::isfinite(cfg.height_max) ||
    cfg.height_min > cfg.height_max ||
    (cfg.enable_low_band && (!std::isfinite(cfg.low_height_min) ||
    !std::isfinite(cfg.low_height_max) || cfg.low_height_min > cfg.low_height_max)) ||
    !projection_detail::floatOffset(msg, "x", ox) ||
    !projection_detail::floatOffset(msg, "y", oy) ||
    !projection_detail::floatOffset(msg, "z", oz)) {return false;}
  const auto & body = cfg.body;
  if (!std::isfinite(body.frame_front) || !std::isfinite(body.frame_back) ||
    !std::isfinite(body.frame_left) || !std::isfinite(body.frame_right) ||
    body.frame_front < 0 || body.frame_back < 0 || body.frame_left < 0 || body.frame_right < 0)
  {return false;}
  for (double v : tf.rotation) {if (!std::isfinite(v)) {return false;}}
  for (double v : tf.translation) {if (!std::isfinite(v)) {return false;}}
  const uint64_t packed_row = uint64_t(msg.width) * msg.point_step;
  const uint64_t total_bytes = uint64_t(msg.height) * msg.row_step;
  if (packed_row > msg.row_step || total_bytes > msg.data.size() ||
    (msg.width > 0 && msg.height == 0)) {return false;}

  out.stamp = rclcpp::Time(msg.header.stamp);
  out.frame_id = control_frame.empty() ? msg.header.frame_id : control_frame;
  out.angle_min = -M_PI;
  out.angle_increment = (2.0 * M_PI) / cfg.angle_bins;
  const float inf = std::numeric_limits<float>::infinity();
  out.ranges.assign(static_cast<size_t>(cfg.angle_bins), inf);
  out.low_ranges.assign(static_cast<size_t>(cfg.angle_bins), inf);
  out.valid_bins = out.valid_low_bins = 0;
  std::unique_ptr<projection_detail::DisplayVoxels> voxels;
  if (display) {
    display->clear();
    display->reserve(6000);
    voxels.reset(new projection_detail::DisplayVoxels);
  }
  const double rmin2 = cfg.range_min * cfg.range_min;
  const double rmax2 = cfg.range_max * cfg.range_max;
  for (uint32_t row = 0; row < msg.height; ++row) {
    for (uint32_t col = 0; col < msg.width; ++col) {
      const uint8_t * point = msg.data.data() + size_t(row) * msg.row_step +
        size_t(col) * msg.point_step;
      double x = projection_detail::readFloat(point + ox, msg.is_bigendian);
      double y = projection_detail::readFloat(point + oy, msg.is_bigendian);
      double z = projection_detail::readFloat(point + oz, msg.is_bigendian);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {continue;}
      tf.transform(x, y, z);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
        insideBodyFrame(body, x, y)) {continue;}
      if (display && display->size() < 6000 &&
        std::abs(x) <= std::numeric_limits<float>::max() &&
        std::abs(y) <= std::numeric_limits<float>::max() &&
        std::abs(z) <= std::numeric_limits<float>::max() && voxels->insert(x, y, z))
      {
        display->push_back(PointXYZ{static_cast<float>(x), static_cast<float>(y),
            static_cast<float>(z)});
      }
      const bool target = z >= cfg.height_min && z <= cfg.height_max;
      const bool low = cfg.enable_low_band && z >= cfg.low_height_min && z <= cfg.low_height_max;
      if (!target && !low) {continue;}
      const double r2 = x * x + y * y;
      if (!std::isfinite(r2) || r2 < rmin2 || r2 > rmax2) {continue;}
      const double az = std::atan2(y, x);
      int bin = static_cast<int>((az - out.angle_min) / out.angle_increment);
      if (bin < 0) {bin = 0;}
      if (bin >= cfg.angle_bins) {bin = cfg.angle_bins - 1;}
      const size_t bi = static_cast<size_t>(bin);
      const float range = static_cast<float>(std::sqrt(r2));
      if (target && range < out.ranges[bi]) {
        if (!std::isfinite(out.ranges[bi])) {++out.valid_bins;}
        out.ranges[bi] = range;
      }
      if (low && range < out.low_ranges[bi]) {
        if (!std::isfinite(out.low_ranges[bi])) {++out.valid_low_bins;}
        out.low_ranges[bi] = range;
      }
    }
  }
  return true;
}

}  // namespace rs_follow
#endif  // RS_FOLLOW_POINTCLOUD_SCAN_HPP
