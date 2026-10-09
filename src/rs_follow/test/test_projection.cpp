#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
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
void storeFloat(uint8_t * dst, float value, bool big)
{
  uint32_t bits;
  std::memcpy(&bits, &value, 4);
  for (unsigned i = 0; i < 4; ++i) {
    dst[i] = static_cast<uint8_t>(bits >> (8 * (big ? 3 - i : i)));
  }
}
sensor_msgs::msg::PointCloud2 cloud(
  const std::vector<PointXYZ> & points, uint32_t rows = 1, uint32_t padding = 0,
  bool big = false)
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.frame_id = "sensor";
  msg.header.stamp.sec = 42;
  msg.header.stamp.nanosec = 123;
  msg.height = rows;
  msg.width = static_cast<uint32_t>(points.size()) / rows;
  msg.point_step = 16;
  msg.row_step = msg.width * msg.point_step + padding;
  msg.is_bigendian = big;
  for (uint32_t i = 0; i < 3; ++i) {
    sensor_msgs::msg::PointField f;
    f.name = i == 0 ? "x" : i == 1 ? "y" : "z";
    f.offset = i * 4;
    f.count = 1;
    f.datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields.push_back(f);
  }
  msg.data.assign(static_cast<size_t>(msg.row_step) * rows, 0xff);
  for (size_t i = 0; i < points.size(); ++i) {
    uint8_t * dst = msg.data.data() + (i / msg.width) * msg.row_step +
      (i % msg.width) * msg.point_step;
    storeFloat(dst, points[i].x, big);
    storeFloat(dst + 4, points[i].y, big);
    storeFloat(dst + 8, points[i].z, big);
  }
  return msg;
}
ProjectionConfig config()
{
  ProjectionConfig cfg;
  cfg.angle_bins = 360;
  cfg.range_min = 0.0;
  cfg.height_min = 0.4;
  cfg.height_max = 1.8;
  cfg.enable_low_band = true;
  cfg.low_height_min = 0.1;
  cfg.low_height_max = 0.3;
  return cfg;
}
bool near(float a, double b) {return std::abs(a - b) < 1e-5;}
bool samePoints(const std::vector<PointXYZ> & a, const std::vector<PointXYZ> & b)
{
  if (a.size() != b.size()) {return false;}
  for (size_t i = 0; i < a.size(); ++i) {
    if (a[i].x != b[i].x || a[i].y != b[i].y || a[i].z != b[i].z) {return false;}
  }
  return true;
}
}

int main()
{
  ProjectionConfig cfg = config();
  ScanFrame scan;
  const auto input = cloud({{2, 0, 1}, {1, 0, 1}, {0.3f, 0, 1}, {0.8f, 0, 0.2f}});
  check(projectPointCloud(input, cfg, scan), "packed input succeeds");
  check(scan.valid_bins == 1 && scan.valid_low_bins == 1 &&
    near(scan.ranges[180], 1) && near(scan.low_ranges[180], 0.8),
    "self rejection precedes minima in both bands");
  check(scan.stamp.nanoseconds() == 42000000123LL && scan.frame_id == "sensor",
    "identity preserves input stamp and frame");

  RigidTransform yaw;
  yaw.rotation = {{0, -1, 0, 1, 0, 0, 0, 0, 1}};
  yaw.translation = {{0, 1, 0.5}};
  std::vector<PointXYZ> display;
  check(projectPointCloud(cloud({{1, 0, 0.5f}}), cfg, scan, yaw, "base_link", &display) &&
    near(scan.ranges[270], 2) && scan.frame_id == "base_link" &&
    display.size() == 1 && near(display[0].y, 2) && near(display[0].z, 1),
    "rigid yaw and translation applied before bin and height tests");
  RigidTransform roll;
  roll.rotation = {{1, 0, 0, 0, 0, -1, 0, 1, 0}};
  check(projectPointCloud(cloud({{1, 1, 0}}), cfg, scan, roll) &&
    near(scan.ranges[180], 1), "roll affects height eligibility");
  RigidTransform offset;
  offset.translation = {{-1, 0, 0}};
  check(projectPointCloud(cloud({{1.2f, 0, 1}, {2, 0, 1}}), cfg, scan, offset) &&
    near(scan.ranges[180], 1), "self mask uses transformed coordinates");

  const auto organized = cloud({{2, 0, 1}, {0, 2, 1}, {-2, 0, 0.2f}, {0, -2, 0.2f}}, 2, 11);
  check(projectPointCloud(organized, cfg, scan) && scan.valid_bins == 2 &&
    scan.valid_low_bins == 2, "organized padded rows skip padding bytes");
  const auto endian = cloud({{2, 0, 1}, {0, 2, 1}, {-2, 0, 0.2f}, {0, -2, 0.2f}}, 2, 11, true);
  ScanFrame big;
  check(projectPointCloud(endian, cfg, big) && big.ranges == scan.ranges &&
    big.low_ranges == scan.low_ranges, "big-endian and little-endian clouds agree");
  auto unaligned = cloud({{2, 0, 1}});
  for (auto & field : unaligned.fields) {++field.offset;}
  storeFloat(unaligned.data.data() + 1, 2, false);
  storeFloat(unaligned.data.data() + 5, 0, false);
  storeFloat(unaligned.data.data() + 9, 1, false);
  check(projectPointCloud(unaligned, cfg, scan) && near(scan.ranges[180], 2),
    "unaligned float offsets are decoded safely");

  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  check(projectPointCloud(cloud({{nan, 0, 1}, {1, inf, 1}, {1, 0, nan}, {2, 0, 1}}),
    cfg, scan) && scan.valid_bins == 1 && near(scan.ranges[180], 2),
    "non-finite points are skipped independent of is_dense");
  check(projectPointCloud(cloud({}), cfg, scan, RigidTransform{}, "base_link", &display) &&
    scan.valid_bins == 0 && scan.valid_low_bins == 0 && display.empty(),
    "well-formed empty cloud clears scans and display");

  auto expectInvalid = [&](sensor_msgs::msg::PointCloud2 msg, const char * label) {
      scan.frame_id = "unchanged";
      scan.ranges = {7};
      display = {{7, 8, 9}};
      check(!projectPointCloud(msg, cfg, scan, RigidTransform{}, "base_link", &display) &&
        scan.frame_id == "unchanged" && scan.ranges == std::vector<float>{7} &&
        display.size() == 1 && display[0].x == 7, label);
    };
  auto bad = input;
  bad.fields.pop_back();
  expectInvalid(bad, "missing XYZ rejected transactionally");
  bad = input; bad.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT64;
  expectInvalid(bad, "non-FLOAT32 XYZ rejected");
  bad = input; bad.fields[0].count = 0;
  expectInvalid(bad, "zero field count rejected");
  bad = input; bad.fields[0].offset = bad.point_step - 2;
  expectInvalid(bad, "field extending beyond point rejected");
  bad = input; bad.fields.push_back(bad.fields[0]);
  expectInvalid(bad, "duplicate XYZ rejected");
  bad = input; --bad.row_step;
  expectInvalid(bad, "row stride smaller than packed row rejected");
  bad = organized; bad.data.pop_back();
  expectInvalid(bad, "truncated organized cloud rejected");
  bad = input; bad.height = std::numeric_limits<uint32_t>::max();
  expectInvalid(bad, "overflow-sized layout rejected");
  bad = input; bad.height = 0;
  expectInvalid(bad, "nonempty width without rows rejected");
  bad = input; bad.header.stamp.sec = -1;
  expectInvalid(bad, "negative ROS timestamp rejected");
  bad = input; bad.header.stamp.nanosec = 1000000000U;
  expectInvalid(bad, "unnormalized ROS timestamp rejected");
  cfg.angle_bins = 0;
  expectInvalid(input, "invalid bin configuration rejected");
  cfg = config(); cfg.range_min = nan;
  expectInvalid(input, "non-finite range configuration rejected");
  cfg = config(); cfg.height_max = 0;
  expectInvalid(input, "inverted height interval rejected");
  cfg = config();

  BodyFrameConfig manual;
  manual.auto_frame = false;
  manual.frame_front = 0.2;
  const auto normalized = normalizeBodyFrame(BodyFrameConfig{});
  check(std::abs(normalized.frame_front - 0.36) < 1e-12 &&
    std::abs(normalized.frame_left - 0.23) < 1e-12 &&
    normalizeBodyFrame(manual).frame_front == 0.2,
    "shared normalization preserves automatic and manual semantics");
  FollowConfig follow;
  follow.robot_length = 1.4;
  follow.robot_width = 0.8;
  FollowController controller(follow);
  const auto body = normalizeBodyFrame(follow);
  cfg.body = body;
  check(projectPointCloud(cloud({{0.6f, 0, 1}, {1, 0, 1}}), cfg, scan) &&
    near(scan.ranges[180], 1), "custom normalized footprint excludes chassis before minima");
  cfg.body = manual;
  check(projectPointCloud(cloud({{0.3f, 0, 1}}), cfg, scan) && scan.valid_bins == 1,
    "manual footprint is used without a second normalization");
  check(!insideBodyFrame(manual, manual.frame_front, 0),
    "self box preserves strict boundary policy");
  cfg = config();
  cfg.range_min = 1; cfg.range_max = 2;
  check(projectPointCloud(cloud({{0.9f, 0, 1}, {1, 0, 1}, {2, 0, 0.2f}, {3, 0, 0.2f}}),
    cfg, scan) && near(scan.ranges[180], 1) && near(scan.low_ranges[180], 2),
    "range endpoints inclusive in both bands");
  cfg.enable_low_band = false;
  check(projectPointCloud(cloud({{2, 0, 0.2f}}), cfg, scan) && scan.valid_low_bins == 0 &&
    scan.valid_bins == 0, "disabled low band never creates obstacle minima");
  cfg = config();
  check(projectPointCloud(cloud({{2, 0, 5}, {2.01f, 0, 5}, {2.2f, 0, 5},
    {-2.01f, 0, 5}, {-2.02f, 0, 5}, {0.1f, 0, 5}}), cfg, scan,
    RigidTransform{}, "base_link", &display) && display.size() == 3 &&
    scan.valid_bins == 0 && near(display[0].x, 2) && near(display[2].x, -2.01),
    "display voxels retain first point, floor negative cells, omit self, include outside bands");
  check(controller.config().frame_front == body.frame_front &&
    controller.config().frame_left == body.frame_left,
    "controller constructor uses same normalized bounds as projection");
  follow.auto_frame = false;
  follow.frame_front = 0.17;
  controller.setConfig(follow);
  check(controller.config().frame_front == normalizeBodyFrame(follow).frame_front,
    "controller updates use shared manual normalization");
  std::vector<PointXYZ> many;
  for (int i = 0; i < 7000; ++i) {many.push_back({1.0f + i * 0.2f, 0, 1});}
  const auto large = cloud(many);
  check(projectPointCloud(large, cfg, scan, RigidTransform{}, "base_link", &display) &&
    display.size() == 6000, "display collection capped at 6000");
  std::vector<PointXYZ> repeated;
  check(projectPointCloud(large, cfg, scan, RigidTransform{}, "base_link", &repeated) &&
    samePoints(display, repeated), "voxel output deterministic across calls");
  std::printf("Projection failures: %d\n", failures);
  return failures == 0 ? 0 : 1;
}
