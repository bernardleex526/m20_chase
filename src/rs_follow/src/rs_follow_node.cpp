/**
 * @file rs_follow_node.cpp
 * @brief ROS 2 node: RoboSense 3D LiDAR point-cloud target following.
 *
 * Subscribes to a PointCloud2 topic (e.g. /rslidar_points), lets the user bind
 * a target point (RViz "Publish Point" -> /clicked_point, or ~/bind_target),
 * tracks it and publishes geometry_msgs/Twist on /cmd_vel.
 *
 * Control law adapted from jie_deamon (MIT), https://github.com/6-robot/jie_deamon
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "rs_follow/cmd_smoother.hpp"
#include "rs_follow/follow_controller.hpp"
#include "rs_follow/pointcloud_scan.hpp"

namespace rs_follow
{

class RsFollowNode : public rclcpp::Node
{
public:
  RsFollowNode()
  : rclcpp::Node("rs_follow_node")
  {
    loadParams();
    controller_.setConfig(follow_cfg_);
    smoother_.setConfig(smoother_cfg_);

    auto sensor_qos = rclcpp::SensorDataQoS();

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, sensor_qos,
      std::bind(&RsFollowNode::cloudCallback, this, std::placeholders::_1));

    clicked_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/clicked_point", 10,
      std::bind(&RsFollowNode::clickedCallback, this, std::placeholders::_1));

    if (!odom_topic_.empty()) {
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10,
        std::bind(&RsFollowNode::odomCallback, this, std::placeholders::_1));
    }

    bind_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/rs_follow/bind_target", 10,
      std::bind(&RsFollowNode::clickedCallback, this, std::placeholders::_1));

    clear_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/rs_follow/clear_target", 10,
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
          controller_.clearTarget();
          RCLCPP_INFO(get_logger(), "target cleared");
        }
      });

    enable_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/rs_follow/enable", 10,
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        active_ = msg->data;
        RCLCPP_INFO(get_logger(), "follow %s", active_ ? "ENABLED" : "DISABLED");
      });

    // direct-control mode (0 = DIRECT joystick, 1 = FOLLOW)
    mode_sub_ = create_subscription<std_msgs::msg::Int32>(
      "/rs_follow/control_mode", 10,
      [this](std_msgs::msg::Int32::SharedPtr msg) {
        control_mode_ = msg->data;
        RCLCPP_INFO(get_logger(), "control_mode -> %s",
                    control_mode_ == 0 ? "DIRECT" : "FOLLOW");
      });
    direct_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/rs_follow/direct_cmd", 10,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        direct_vx_ = msg->linear.x;
        direct_vy_ = msg->linear.y;
        direct_wz_ = msg->angular.z;
      });

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/rs_follow/scan", 10);
    target_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/rs_follow/target", 10);
    target_raw_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/rs_follow/target_raw", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/rs_follow/status", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/rs_follow/target_marker", 10);

    last_result_ = FollowResult();
    last_cloud_time_ = now();

    const double period = 1.0 / std::max(1.0, control_rate_hz_);
    control_timer_ = create_wall_timer(
      std::chrono::duration<double>(period),
      std::bind(&RsFollowNode::controlTimer, this));

    RCLCPP_INFO(
      get_logger(),
      "rs_follow ready | cloud='%s' cmd_vel='%s' auto_front=%s follow_dist=%.2f",
      input_topic_.c_str(), cmd_vel_topic_.c_str(),
      follow_cfg_.auto_select_front ? "on" : "off", follow_cfg_.follow_dist);
  }

private:
  void loadParams()
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/rslidar_points");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    odom_topic_ = declare_parameter<std::string>("odom_topic", odom_topic_);
    compensate_slip_ = declare_parameter<bool>("compensate_slip", compensate_slip_);
    slip_min_ratio_ = declare_parameter<double>("slip_min_ratio", slip_min_ratio_);
    slip_max_ratio_ = declare_parameter<double>("slip_max_ratio", slip_max_ratio_);
    slip_filter_alpha_ = declare_parameter<double>("slip_filter_alpha", slip_filter_alpha_);
    slip_min_cmd_ = declare_parameter<double>("slip_min_cmd", slip_min_cmd_);
    max_linear_cmd_ = declare_parameter<double>("max_linear_cmd", max_linear_cmd_);
    max_angular_cmd_ = declare_parameter<double>("max_angular_cmd", max_angular_cmd_);
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 50.0);
    cmd_timeout_ = declare_parameter<double>("cmd_timeout", 0.5);
    active_ = declare_parameter<bool>("active", false);
    publish_scan_debug_ = declare_parameter<bool>("publish_scan_debug", true);

    proj_cfg_.height_min = declare_parameter<double>("height_min", proj_cfg_.height_min);
    proj_cfg_.height_max = declare_parameter<double>("height_max", proj_cfg_.height_max);
    proj_cfg_.z_offset = declare_parameter<double>("z_offset", proj_cfg_.z_offset);
    proj_cfg_.angle_bins = declare_parameter<int>("angle_bins", proj_cfg_.angle_bins);
    proj_cfg_.range_min = declare_parameter<double>("range_min", proj_cfg_.range_min);
    proj_cfg_.range_max = declare_parameter<double>("range_max", proj_cfg_.range_max);
    proj_cfg_.flip_x = declare_parameter<bool>("flip_x", proj_cfg_.flip_x);
    proj_cfg_.flip_y = declare_parameter<bool>("flip_y", proj_cfg_.flip_y);

    follow_cfg_.follow_dist = declare_parameter<double>("follow_dist", follow_cfg_.follow_dist);
    follow_cfg_.target_radius = declare_parameter<double>("target_radius", follow_cfg_.target_radius);
    follow_cfg_.rectangle_width =
      declare_parameter<double>("rectangle_width", follow_cfg_.rectangle_width);
    follow_cfg_.k_linear = declare_parameter<double>("k_linear", follow_cfg_.k_linear);
    follow_cfg_.k_angular = declare_parameter<double>("k_angular", follow_cfg_.k_angular);
    follow_cfg_.k_lateral = declare_parameter<double>("k_lateral", follow_cfg_.k_lateral);
    follow_cfg_.enable_lateral =
      declare_parameter<bool>("enable_lateral", follow_cfg_.enable_lateral);
    follow_cfg_.rotate_in_place_behind =
      declare_parameter<bool>("rotate_in_place_behind", follow_cfg_.rotate_in_place_behind);
    follow_cfg_.max_linear = declare_parameter<double>("max_linear", follow_cfg_.max_linear);
    follow_cfg_.max_angular = declare_parameter<double>("max_angular", follow_cfg_.max_angular);
    follow_cfg_.linear_deadband =
      declare_parameter<double>("linear_deadband", follow_cfg_.linear_deadband);
    follow_cfg_.angular_deadband =
      declare_parameter<double>("angular_deadband", follow_cfg_.angular_deadband);
    follow_cfg_.min_linear_speed =
      declare_parameter<double>("min_linear_speed", follow_cfg_.min_linear_speed);
    follow_cfg_.linear_hysteresis =
      declare_parameter<double>("linear_hysteresis", follow_cfg_.linear_hysteresis);
    follow_cfg_.angular_hysteresis =
      declare_parameter<double>("angular_hysteresis", follow_cfg_.angular_hysteresis);
    follow_cfg_.k_integral = declare_parameter<double>("k_integral", follow_cfg_.k_integral);
    follow_cfg_.integral_limit =
      declare_parameter<double>("integral_limit", follow_cfg_.integral_limit);
    follow_cfg_.apf_influence = declare_parameter<double>("apf_influence", follow_cfg_.apf_influence);
    follow_cfg_.apf_gain = declare_parameter<double>("apf_gain", follow_cfg_.apf_gain);
    follow_cfg_.apf_emergency = declare_parameter<double>("apf_emergency", follow_cfg_.apf_emergency);
    follow_cfg_.apf_slowdown = declare_parameter<double>("apf_slowdown", follow_cfg_.apf_slowdown);
    follow_cfg_.frame_front = declare_parameter<double>("frame_front", follow_cfg_.frame_front);
    follow_cfg_.frame_back = declare_parameter<double>("frame_back", follow_cfg_.frame_back);
    follow_cfg_.frame_left = declare_parameter<double>("frame_left", follow_cfg_.frame_left);
    follow_cfg_.frame_right = declare_parameter<double>("frame_right", follow_cfg_.frame_right);
    follow_cfg_.enable_kalman = declare_parameter<bool>("enable_kalman", follow_cfg_.enable_kalman);
    follow_cfg_.kalman_q = declare_parameter<double>("kalman_q", follow_cfg_.kalman_q);
    follow_cfg_.kalman_r = declare_parameter<double>("kalman_r", follow_cfg_.kalman_r);
    follow_cfg_.kalman_gate = declare_parameter<double>("kalman_gate", follow_cfg_.kalman_gate);
    follow_cfg_.kalman_r_ref_points =
      declare_parameter<int>("kalman_r_ref_points", follow_cfg_.kalman_r_ref_points);
    follow_cfg_.filter_in_world =
      declare_parameter<bool>("filter_in_world", follow_cfg_.filter_in_world);
    follow_cfg_.lost_frames_timeout =
      declare_parameter<int>("lost_frames_timeout", follow_cfg_.lost_frames_timeout);
    follow_cfg_.auto_select_front =
      declare_parameter<bool>("auto_select_front", follow_cfg_.auto_select_front);
    follow_cfg_.auto_front_fov_deg =
      declare_parameter<double>("auto_front_fov_deg", follow_cfg_.auto_front_fov_deg);
    follow_cfg_.auto_select_max_range =
      declare_parameter<double>("auto_select_max_range", follow_cfg_.auto_select_max_range);
    follow_cfg_.bind_radius = declare_parameter<double>("bind_radius", follow_cfg_.bind_radius);

    smoother_cfg_.max_linear_accel =
      declare_parameter<double>("max_linear_accel", smoother_cfg_.max_linear_accel);
    smoother_cfg_.max_angular_accel =
      declare_parameter<double>("max_angular_accel", smoother_cfg_.max_angular_accel);
    smoother_cfg_.cmd_filter_alpha =
      declare_parameter<double>("cmd_filter_alpha", smoother_cfg_.cmd_filter_alpha);
    smoother_cfg_.max_linear_jerk =
      declare_parameter<double>("max_linear_jerk", smoother_cfg_.max_linear_jerk);
    smoother_cfg_.max_angular_jerk =
      declare_parameter<double>("max_angular_jerk", smoother_cfg_.max_angular_jerk);
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    ScanFrame scan;
    if (!projectPointCloud(*msg, proj_cfg_, scan)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "cannot project cloud (missing x/y/z FLOAT32 fields?) topic=%s", input_topic_.c_str());
      return;
    }

    last_scan_ = scan;
    last_scan_valid_ = true;
    last_result_ = controller_.update(scan);
    last_cloud_time_ = now();

    if (publish_scan_debug_) {
      publishScan(scan);
    }
    publishTarget(scan);
    publishStatus();

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "target=%s range=%.2f bearing=%.1fdeg pts=%d min_obs=%.2f cmd=(%.2f,%.2f,%.2f)",
      last_result_.target_valid ? "LOCK" : "NONE",
      last_result_.target_range, last_result_.target_bearing * 180.0 / M_PI,
      last_result_.points_in_target,
      std::isfinite(last_result_.min_obstacle_dist) ? last_result_.min_obstacle_dist : -1.0,
      last_result_.cmd.linear.x, last_result_.cmd.linear.y, last_result_.cmd.angular.z);
  }

  void clickedCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    if (!last_scan_valid_) {
      RCLCPP_WARN(get_logger(), "no scan yet, cannot bind target");
      return;
    }
    const bool snapped = controller_.bindTarget(msg->point.x, msg->point.y, last_scan_);
    RCLCPP_INFO(
      get_logger(), "bind target (%.2f, %.2f) snapped=%s -> (%.2f, %.2f)",
      msg->point.x, msg->point.y, snapped ? "yes" : "no",
      controller_.targetX(), controller_.targetY());
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto & p = msg->pose.pose.position;
    const auto & q = msg->pose.pose.orientation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    controller_.setOdomPose(p.x, p.y, yaw);

    if (compensate_slip_) {
      const auto & tw = msg->twist.twist;
      const double cmdmag = std::hypot(last_pub_vx_, last_pub_vy_);
      const double actmag = std::hypot(tw.linear.x, tw.linear.y);
      if (cmdmag > slip_min_cmd_) {
        const double r = std::clamp(actmag / cmdmag, slip_min_ratio_, slip_max_ratio_);
        lin_ratio_ += slip_filter_alpha_ * (r - lin_ratio_);
      }
      if (std::abs(last_pub_wz_) > slip_min_cmd_) {
        const double r = std::clamp(std::abs(tw.angular.z) / std::abs(last_pub_wz_),
                                    slip_min_ratio_, slip_max_ratio_);
        ang_ratio_ += slip_filter_alpha_ * (r - ang_ratio_);
      }
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "slip comp: lin_ratio=%.2f ang_ratio=%.2f (actual/cmd)", lin_ratio_, ang_ratio_);
    }
  }

  void controlTimer()
  {
    const auto now_tp = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(now_tp - last_tick_time_).count();
    last_tick_time_ = now_tp;

    const double age = (now() - last_cloud_time_).seconds();
    const bool fresh = age < cmd_timeout_;

    geometry_msgs::msg::Twist desired;  // zero unless a valid, fresh command exists
    bool emergency = false;
    if (active_ && fresh) {
      if (control_mode_ == 0) {                 // DIRECT (joystick)
        desired.linear.x = direct_vx_;
        desired.linear.y = direct_vy_;
        desired.angular.z = direct_wz_;
      } else if (last_result_.target_valid) {   // FOLLOW
        if (last_result_.emergency_stop) {
          emergency = true;
        } else {
          desired = last_result_.cmd;
        }
      }
    }

    // slip / speed compensation (follow only)
    if (compensate_slip_ && !emergency && control_mode_ != 0) {
      desired.linear.x /= lin_ratio_;
      desired.linear.y /= lin_ratio_;
      desired.angular.z /= ang_ratio_;
      desired.linear.x = std::clamp(desired.linear.x, -max_linear_cmd_, max_linear_cmd_);
      desired.linear.y = std::clamp(desired.linear.y, -max_linear_cmd_, max_linear_cmd_);
      desired.angular.z = std::clamp(desired.angular.z, -max_angular_cmd_, max_angular_cmd_);
    }

    const geometry_msgs::msg::Twist out = smoother_.step(desired, dt, emergency);
    // feed the actually-commanded motion back so the target filter can work in
    // an inertial frame (compensates the robot's own rotation)
    controller_.setOdomTwist(out.linear.x, out.linear.y, out.angular.z);
    cmd_pub_->publish(out);

    last_pub_vx_ = out.linear.x;
    last_pub_vy_ = out.linear.y;
    last_pub_wz_ = out.angular.z;
  }

  void publishScan(const ScanFrame & scan)
  {
    sensor_msgs::msg::LaserScan out;
    out.header.stamp = scan.stamp;
    out.header.frame_id = scan.frame_id;
    out.angle_min = static_cast<float>(scan.angle_min);
    out.angle_max = static_cast<float>(scan.angle_min + scan.angle_increment * (scan.ranges.size() - 1));
    out.angle_increment = static_cast<float>(scan.angle_increment);
    out.range_min = static_cast<float>(proj_cfg_.range_min);
    out.range_max = static_cast<float>(proj_cfg_.range_max);
    out.ranges = scan.ranges;
    scan_pub_->publish(out);
  }

  void publishTarget(const ScanFrame & scan)
  {
    if (!last_result_.target_valid) {
      return;
    }
    geometry_msgs::msg::PointStamped pt;
    pt.header.stamp = scan.stamp;
    pt.header.frame_id = scan.frame_id;
    pt.point.x = last_result_.target_x;
    pt.point.y = last_result_.target_y;
    pt.point.z = 0.0;
    target_pub_->publish(pt);

    geometry_msgs::msg::PointStamped raw;
    raw.header = pt.header;
    raw.point.x = last_result_.target_raw_x;
    raw.point.y = last_result_.target_raw_y;
    raw.point.z = 0.0;
    target_raw_pub_->publish(raw);

    visualization_msgs::msg::Marker m;
    m.header = pt.header;
    m.ns = "rs_follow";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position = pt.point;
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = 0.3;
    m.color.a = 1.0;
    m.color.r = last_result_.target_manual ? 0.1f : 0.1f;
    m.color.g = last_result_.target_manual ? 0.9f : 0.5f;
    m.color.b = last_result_.target_manual ? 0.1f : 0.9f;
    marker_pub_->publish(m);
  }

  void publishStatus()
  {
    std_msgs::msg::String s;
    s.data = last_result_.target_valid ?
      (last_result_.emergency_stop ? "EMERGENCY_STOP" :
      (last_result_.target_manual ? "TRACKING_MANUAL" : "TRACKING_AUTO")) : "NO_TARGET";
    status_pub_->publish(s);
    last_scan_valid_ = true;
  }

  // topics
  std::string input_topic_;
  std::string cmd_vel_topic_;
  std::string odom_topic_ = "/odom";

  // runtime
  ProjectionConfig proj_cfg_;
  FollowConfig follow_cfg_;
  SmootherConfig smoother_cfg_;
  FollowController controller_;
  CmdSmoother smoother_;
  FollowResult last_result_;
  ScanFrame last_scan_;
  bool last_scan_valid_ = false;
  bool active_ = false;
  bool publish_scan_debug_ = true;
  int control_mode_ = 1;                       // 0 = DIRECT, 1 = FOLLOW
  double direct_vx_ = 0.0, direct_vy_ = 0.0, direct_wz_ = 0.0;
  double control_rate_hz_ = 50.0;
  double cmd_timeout_ = 0.5;
  rclcpp::Time last_cloud_time_;
  std::chrono::steady_clock::time_point last_tick_time_ = std::chrono::steady_clock::now();

  // slip / speed compensation
  bool compensate_slip_ = true;
  double slip_min_ratio_ = 0.3;
  double slip_max_ratio_ = 2.5;
  double slip_filter_alpha_ = 0.1;
  double slip_min_cmd_ = 0.05;
  double max_linear_cmd_ = 1.5;
  double max_angular_cmd_ = 2.0;
  double lin_ratio_ = 1.0;
  double ang_ratio_ = 1.0;
  double last_pub_vx_ = 0.0;
  double last_pub_vy_ = 0.0;
  double last_pub_wz_ = 0.0;

  // ROS
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr bind_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr clear_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr direct_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_raw_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace rs_follow

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rs_follow::RsFollowNode>());
  rclcpp::shutdown();
  return 0;
}
