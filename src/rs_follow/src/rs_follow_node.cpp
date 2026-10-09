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
#include <cstdio>
#include <cmath>
#include <memory>
#include <string>
#include <stdexcept>
#include <cstdint>
#include <cstring>
#include <vector>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>

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
#include "rs_follow/binding.hpp"
#include "rs_follow/recovery_fsm.hpp"
#include "rs_follow_interfaces/srv/bind_target.hpp"

namespace rs_follow
{

class RsFollowNode : public rclcpp::Node
{
public:
  RsFollowNode()
  : rclcpp::Node("rs_follow_node")
  {
    loadParams();
    if (!std::isfinite(cmd_timeout_) || cmd_timeout_ <= 0.0 ||
      !std::isfinite(direct_cmd_timeout_) || direct_cmd_timeout_ <= 0.0 ||
      !std::isfinite(target_observation_timeout_) || target_observation_timeout_ <= 0.0 ||
      !std::isfinite(follow_cfg_.max_linear) || follow_cfg_.max_linear <= 0.0 ||
      !std::isfinite(follow_cfg_.max_angular) || follow_cfg_.max_angular <= 0.0)
    {throw std::invalid_argument("watchdog timeouts and velocity limits must be finite and positive");}
    if (control_frame_.empty()) {throw std::invalid_argument("control_frame must not be empty");}
    proj_cfg_.body = normalizeBodyFrame(follow_cfg_);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    controller_.setConfig(follow_cfg_);
    recovery_.setConfig(recovery_cfg_);
    search_.setConfig(search_cfg_);
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
          active_ = false;
          have_target_observation_ = false;
          last_result_ = FollowResult();
          stopNow();
          RCLCPP_INFO(get_logger(), "target cleared");
        }
      });

    enable_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/rs_follow/enable", 10,
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        active_ = msg->data && !estop_;
        if (!active_) {stopNow();}
        RCLCPP_INFO(get_logger(), "follow %s", active_ ? "ENABLED" : "DISABLED");
      });

    // direct-control mode (0 = DIRECT joystick, 1 = FOLLOW)
    mode_sub_ = create_subscription<std_msgs::msg::Int32>(
      "/rs_follow/control_mode", 10,
      [this](std_msgs::msg::Int32::SharedPtr msg) {
        if (msg->data != 0 && msg->data != 1) {return;}
        if (control_mode_ != msg->data) {stopNow();}
        control_mode_ = msg->data;
        RCLCPP_INFO(get_logger(), "control_mode -> %s",
                    control_mode_ == 0 ? "DIRECT" : "FOLLOW");
      });
    direct_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/rs_follow/direct_cmd", 10,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        if (estop_ || !std::isfinite(msg->linear.x) ||
          !std::isfinite(msg->linear.y) || !std::isfinite(msg->angular.z))
        {
          stopNow();
          return;
        }
        direct_vx_ = std::clamp(msg->linear.x, -follow_cfg_.max_linear, follow_cfg_.max_linear);
        direct_vy_ = follow_cfg_.enable_lateral ?
          std::clamp(msg->linear.y, -follow_cfg_.max_linear, follow_cfg_.max_linear) : 0.0;
        direct_wz_ = std::clamp(msg->angular.z, -follow_cfg_.max_angular, follow_cfg_.max_angular);
        have_direct_ = true;
        last_direct_time_ = std::chrono::steady_clock::now();
      });

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/rs_follow/scan", 10);
    cloud_viz_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/rs_follow/cloud_viz", 1);
    target_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/rs_follow/target", 10);
    target_raw_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/rs_follow/target_raw", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/rs_follow/status", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/rs_follow/target_marker", 10);
    state_pub_ = create_publisher<std_msgs::msg::String>("/rs_follow/state", 10);
    control_state_pub_ = create_publisher<std_msgs::msg::String>("/rs_follow/control_state", 10);
    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/rs_follow/estop", rclcpp::QoS(1).reliable().durability_volatile(),
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        estop_ = msg->data;
        active_ = false;
        if (estop_) {
          controller_.clearTarget();
          have_target_observation_ = false;
          last_result_ = FollowResult();
        }
        stopNow();
      });
    bind_service_ = create_service<rs_follow_interfaces::srv::BindTarget>(
      "/rs_follow/bind", [this](
        const std::shared_ptr<rs_follow_interfaces::srv::BindTarget::Request> request,
        std::shared_ptr<rs_follow_interfaces::srv::BindTarget::Response> response) {
        response->reason = validateBinding(request->point, response->target);
        response->success = response->reason == "OK";
      });

    last_result_ = FollowResult();

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
    control_frame_ = declare_parameter<std::string>("control_frame", "base_link");
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
    direct_cmd_timeout_ = declare_parameter<double>("direct_cmd_timeout", 0.3);
    target_observation_timeout_ = declare_parameter<double>("target_observation_timeout", 0.5);
    active_ = declare_parameter<bool>("active", false);
    publish_scan_debug_ = declare_parameter<bool>("publish_scan_debug", true);

    proj_cfg_.height_min = declare_parameter<double>("height_min", proj_cfg_.height_min);
    proj_cfg_.height_max = declare_parameter<double>("height_max", proj_cfg_.height_max);
    proj_cfg_.angle_bins = declare_parameter<int>("angle_bins", proj_cfg_.angle_bins);
    proj_cfg_.range_min = declare_parameter<double>("range_min", proj_cfg_.range_min);
    proj_cfg_.range_max = declare_parameter<double>("range_max", proj_cfg_.range_max);
    proj_cfg_.enable_low_band =
      declare_parameter<bool>("enable_low_band", proj_cfg_.enable_low_band);
    proj_cfg_.low_height_min =
      declare_parameter<double>("low_height_min", proj_cfg_.low_height_min);
    proj_cfg_.low_height_max =
      declare_parameter<double>("low_height_max", proj_cfg_.low_height_max);

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
      declare_parameter<bool>("auto_select_front", false);
    follow_cfg_.auto_front_fov_deg =
      declare_parameter<double>("auto_front_fov_deg", follow_cfg_.auto_front_fov_deg);
    follow_cfg_.auto_select_max_range =
      declare_parameter<double>("auto_select_max_range", follow_cfg_.auto_select_max_range);
    follow_cfg_.auto_max_target_width =
      declare_parameter<double>("auto_max_target_width",
                                follow_cfg_.auto_max_target_width);
    follow_cfg_.bind_radius = declare_parameter<double>("bind_radius", follow_cfg_.bind_radius);
    follow_cfg_.exclude_target_from_obstacles =
      declare_parameter<bool>("exclude_target_from_obstacles",
                              follow_cfg_.exclude_target_from_obstacles);
    follow_cfg_.target_exclude_slack =
      declare_parameter<double>("target_exclude_slack", follow_cfg_.target_exclude_slack);
    follow_cfg_.direction_gated_obstacles =
      declare_parameter<bool>("direction_gated_obstacles",
                              follow_cfg_.direction_gated_obstacles);
    follow_cfg_.sector_half_angle_deg =
      declare_parameter<double>("sector_half_angle_deg", follow_cfg_.sector_half_angle_deg);
    follow_cfg_.robot_length =
      declare_parameter<double>("robot_length", follow_cfg_.robot_length);
    follow_cfg_.robot_width =
      declare_parameter<double>("robot_width", follow_cfg_.robot_width);
    follow_cfg_.behind_rotate_timeout =
      declare_parameter<double>("behind_rotate_timeout", follow_cfg_.behind_rotate_timeout);
    follow_cfg_.auto_frame =
      declare_parameter<bool>("auto_frame", follow_cfg_.auto_frame);
    follow_cfg_.self_occlusion_margin =
      declare_parameter<double>("self_occlusion_margin", follow_cfg_.self_occlusion_margin);
    follow_cfg_.governor.emergency_floor =
      declare_parameter<double>("governor_emergency_floor",
                                follow_cfg_.governor.emergency_floor);

    // --- gap-based local steering (VFH+) ---
    follow_cfg_.vfh.enable =
      declare_parameter<bool>("vfh_enable", follow_cfg_.vfh.enable);
    follow_cfg_.vfh.bins =
      declare_parameter<int>("vfh_bins", follow_cfg_.vfh.bins);
    follow_cfg_.vfh.d_safe =
      declare_parameter<double>("vfh_d_safe", follow_cfg_.vfh.d_safe);
    follow_cfg_.vfh.auto_d_safe =
      declare_parameter<bool>("vfh_auto_d_safe", follow_cfg_.vfh.auto_d_safe);
    follow_cfg_.vfh.d_safe_margin =
      declare_parameter<double>("vfh_d_safe_margin", follow_cfg_.vfh.d_safe_margin);
    follow_cfg_.vfh.goal_weight =
      declare_parameter<double>("vfh_goal_weight", follow_cfg_.vfh.goal_weight);
    follow_cfg_.vfh.hysteresis_deg =
      declare_parameter<double>("vfh_hysteresis_deg", follow_cfg_.vfh.hysteresis_deg);
    follow_cfg_.vfh.max_turn_deg =
      declare_parameter<double>("vfh_max_turn_deg", follow_cfg_.vfh.max_turn_deg);

    // --- P2 recovery / target search ---
    recovery_cfg_.enable = declare_parameter<bool>("recovery_enable", false);
    recovery_cfg_.stuck_window =
      declare_parameter<double>("recovery_stuck_window", recovery_cfg_.stuck_window);
    recovery_cfg_.stuck_dist =
      declare_parameter<double>("recovery_stuck_dist", recovery_cfg_.stuck_dist);
    recovery_cfg_.backup_dist =
      declare_parameter<double>("recovery_backup_dist", recovery_cfg_.backup_dist);
    recovery_cfg_.backup_speed =
      declare_parameter<double>("recovery_backup_speed", recovery_cfg_.backup_speed);
    recovery_cfg_.spin_rate =
      declare_parameter<double>("recovery_spin_rate", recovery_cfg_.spin_rate);
    recovery_cfg_.spin_timeout =
      declare_parameter<double>("recovery_spin_timeout", recovery_cfg_.spin_timeout);
    recovery_cfg_.max_attempts =
      declare_parameter<int>("recovery_max_attempts", recovery_cfg_.max_attempts);

    search_cfg_.enable = declare_parameter<bool>("search_enable", false);
    search_cfg_.coast_time =
      declare_parameter<double>("search_coast_time", search_cfg_.coast_time);
    search_cfg_.spin_timeout =
      declare_parameter<double>("search_spin_timeout", search_cfg_.spin_timeout);
    search_cfg_.go_speed =
      declare_parameter<double>("search_go_speed", search_cfg_.go_speed);
    search_cfg_.go_tol = declare_parameter<double>("search_go_tol", search_cfg_.go_tol);
    search_cfg_.go_timeout =
      declare_parameter<double>("search_go_timeout", search_cfg_.go_timeout);
    search_cfg_.search_budget =
      declare_parameter<double>("search_budget", search_cfg_.search_budget);
    search_cfg_.max_sweeps =
      declare_parameter<int>("search_max_sweeps", search_cfg_.max_sweeps);

    // --- TTC speed governor ---
    follow_cfg_.governor.enable =
      declare_parameter<bool>("governor_enable", follow_cfg_.governor.enable);
    follow_cfg_.governor.d_hard =
      declare_parameter<double>("governor_d_hard", follow_cfg_.governor.d_hard);
    follow_cfg_.governor.d_margin =
      declare_parameter<double>("governor_d_margin", follow_cfg_.governor.d_margin);
    follow_cfg_.governor.t_lat =
      declare_parameter<double>("governor_t_lat", follow_cfg_.governor.t_lat);
    follow_cfg_.governor.a_max =
      declare_parameter<double>("governor_a_max", follow_cfg_.governor.a_max);
    follow_cfg_.governor.v_cap =
      declare_parameter<double>("governor_v_cap", follow_cfg_.governor.v_cap);
    follow_cfg_.governor.release_hysteresis =
      declare_parameter<double>("governor_release_hysteresis",
                                follow_cfg_.governor.release_hysteresis);

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

  bool lookupTransform(const std::string & source,
    const builtin_interfaces::msg::Time & stamp, RigidTransform & transform)
  {
    if (source.empty() || stamp.sec < 0 || stamp.nanosec >= 1000000000u ||
      (stamp.sec == 0 && stamp.nanosec == 0)) {return false;}
    transform = RigidTransform{};
    if (source == control_frame_) {return true;}
    try {
      const auto stamped = tf_buffer_->lookupTransform(
        control_frame_, source, rclcpp::Time(stamp, get_clock()->get_clock_type()));
      const auto & q = stamped.transform.rotation;
      const auto & t = stamped.transform.translation;
      if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) ||
        !std::isfinite(q.w) || !std::isfinite(t.x) || !std::isfinite(t.y) ||
        !std::isfinite(t.z)) {return false;}
      tf2::Quaternion rotation(q.x, q.y, q.z, q.w);
      if (rotation.length2() <= 0.0) {return false;}
      rotation.normalize();
      const tf2::Matrix3x3 matrix(rotation);
      for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
          transform.rotation[row * 3 + col] = matrix[row][col];
        }
      }
      transform.translation = {t.x, t.y, t.z};
      return true;
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "stamped TF unavailable: %s", error.what());
      return false;
    }
  }

  void invalidateCloud(const builtin_interfaces::msg::Time & stamp, bool display_due)
  {
    have_cloud_ = false;
    last_scan_valid_ = false;
    last_scan_ = ScanFrame();
    active_ = false;
    have_target_observation_ = false;
    controller_.clearTarget();
    last_result_ = FollowResult();
    stopNow();
    pending_empty_viz_ = true;
    empty_viz_stamp_ = stamp;
    if (display_due) {publishCloudViz(stamp, {});}
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = control_frame_;
    marker.ns = "rs_follow";
    marker.id = 0;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    marker_pub_->publish(marker);
    publishStatus();
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const auto now_tp = std::chrono::steady_clock::now();
    const bool display_due = !pending_empty_viz_ &&
      cloud_viz_pub_->get_subscription_count() > 0 &&
      (!have_viz_publish_ || std::chrono::duration<double>(
      now_tp - last_viz_publish_).count() >= 0.2);
    const auto & stamp = msg->header.stamp;
    const bool valid_stamp = stamp.sec >= 0 && stamp.nanosec < 1000000000u &&
      (stamp.sec != 0 || stamp.nanosec != 0);
    const int64_t stamp_ns = static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
    const bool stamp_gap = have_cloud_stamp_ && stamp_ns - last_cloud_stamp_ns_ > 1000000000LL;
    if (stamp_gap) {
      ScanFrame discontinuity;
      discontinuity.stamp = rclcpp::Time(stamp);
      controller_.update(discontinuity);
      invalidateCloud(stamp, display_due);
      have_cloud_stamp_ = true;
      last_cloud_stamp_ns_ = stamp_ns;
      return;
    }
    RigidTransform transform;
    ScanFrame scan;
    std::vector<PointXYZ> display;
    if (!valid_stamp || (have_cloud_stamp_ && stamp_ns <= last_cloud_stamp_ns_) ||
      !lookupTransform(msg->header.frame_id, stamp, transform) ||
      !projectPointCloud(*msg, proj_cfg_, scan, transform, control_frame_,
      display_due ? &display : nullptr) ||
      (scan.valid_bins == 0 && scan.valid_low_bins == 0))
    {
      invalidateCloud(stamp, display_due);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "rejecting cloud: invalid stamp, stamped TF, layout, or empty projection");
      return;
    }
    have_cloud_stamp_ = true;
    last_cloud_stamp_ns_ = stamp_ns;
    if (display_due) {publishCloudViz(stamp, display);}
    last_scan_ = std::move(scan);
    last_scan_valid_ = true;
    last_result_ = controller_.update(last_scan_);
    have_cloud_ = true;
    last_cloud_time_ = now_tp;
    if (last_result_.target_observed) {
      have_target_observation_ = true;
      last_target_observation_ = last_cloud_time_;
    }

    if (publish_scan_debug_) {
      publishScan(last_scan_);
    }
    publishTarget(last_scan_);
    publishStatus();

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "target=%s range=%.2f bearing=%.1fdeg pts=%d min_obs=%.2f "
      "clr(f/r/u)=%.2f/%.2f/%.2f vlim=%.2f excl=%s cmd=(%.2f,%.2f,%.2f)",
      last_result_.target_valid ? "LOCK" : "NONE",
      last_result_.target_range, last_result_.target_bearing * 180.0 / M_PI,
      last_result_.points_in_target,
      std::isfinite(last_result_.min_obstacle_dist) ? last_result_.min_obstacle_dist : -1.0,
      std::isfinite(last_result_.clearance_front) ? last_result_.clearance_front : -1.0,
      std::isfinite(last_result_.clearance_rear) ? last_result_.clearance_rear : -1.0,
      std::isfinite(last_result_.clearance_used) ? last_result_.clearance_used : -1.0,
      std::isfinite(last_result_.speed_limit) ? last_result_.speed_limit : -1.0,
      last_result_.target_excluded ? "Y" : "n",
      last_result_.cmd.linear.x, last_result_.cmd.linear.y, last_result_.cmd.angular.z);
  }

  static bool finiteTwist(const geometry_msgs::msg::Twist & cmd)
  {
    return std::isfinite(cmd.linear.x) && std::isfinite(cmd.linear.y) &&
           std::isfinite(cmd.linear.z) && std::isfinite(cmd.angular.x) &&
           std::isfinite(cmd.angular.y) && std::isfinite(cmd.angular.z);
  }

  void resetCommands()
  {
    smoother_.reset();
    last_result_.cmd = geometry_msgs::msg::Twist();
    direct_vx_ = direct_vy_ = direct_wz_ = 0.0;
    have_direct_ = false;
    last_pub_vx_ = last_pub_vy_ = last_pub_wz_ = 0.0;
    controller_.setOdomTwist(0.0, 0.0, 0.0);
    recovery_active_ = false;
    recovery_.reset();
    search_.reset();
  }

  void stopNow()
  {
    resetCommands();
    if (cmd_pub_) {cmd_pub_->publish(geometry_msgs::msg::Twist());}
  }

  std::string validateBinding(const geometry_msgs::msg::PointStamped & point,
    geometry_msgs::msg::PointStamped & target)
  {
    const bool fresh = have_cloud_ && std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_cloud_time_).count() <= cmd_timeout_;
    const auto reason = validateBindingPoint(point, last_scan_valid_ ? &last_scan_ : nullptr,
      fresh, cmd_timeout_, control_frame_, proj_cfg_.body,
      [this](const std::string & source, const builtin_interfaces::msg::Time & stamp,
        RigidTransform & transform) {return lookupTransform(source, stamp, transform);},
      controller_, target);
    if (reason != "OK") {return reason;}
    have_target_observation_ = true;
    last_target_observation_ = last_cloud_time_;
    last_result_ = FollowResult();
    stopNow();
    return reason;
  }

  void clickedCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    geometry_msgs::msg::PointStamped target;
    const auto reason = validateBinding(*msg, target);
    RCLCPP_INFO(get_logger(), "bind target: %s", reason.c_str());
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto & p = msg->pose.pose.position;
    const auto & q = msg->pose.pose.orientation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    controller_.setOdomPose(p.x, p.y, yaw);
    pose_x_ = p.x;
    pose_y_ = p.y;
    pose_yaw_ = yaw;
    have_pose_ = true;

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
    if (pending_empty_viz_ && cloud_viz_pub_->get_subscription_count() > 0 &&
      (!have_viz_publish_ || std::chrono::duration<double>(
      now_tp - last_viz_publish_).count() >= 0.2))
    {publishCloudViz(empty_viz_stamp_, {});}
    const double dt = std::chrono::duration<double>(now_tp - last_tick_time_).count();
    last_tick_time_ = now_tp;
    // WallTimer remains live when /clock stops; only motion time follows ROS time.
    const int64_t ros_ns = now().nanoseconds();
    const bool sim_time = get_clock()->ros_time_is_active();
    const double motion_dt = sim_time ? (have_tick_ros_time_ ?
      static_cast<double>(ros_ns - last_tick_ros_ns_) * 1e-9 : 0.0) : dt;
    have_tick_ros_time_ = true;
    last_tick_ros_ns_ = ros_ns;
    const bool time_paused = sim_time && motion_dt <= 0.0;
    const bool time_discontinuous = sim_time && (motion_dt < 0.0 || motion_dt > 1.0);
    if (!sim_time || motion_dt > 0.0) {last_ros_advance_time_ = now_tp;}
    const bool clock_frozen = time_paused && std::chrono::duration<double>(
      now_tp - last_ros_advance_time_).count() > cmd_timeout_;
    if (clock_frozen || time_discontinuous) {
      active_ = false;
      have_cloud_ = false;
      last_scan_valid_ = false;
      last_scan_ = ScanFrame();
      have_target_observation_ = false;
      controller_.clearTarget();
      last_result_ = FollowResult();
      resetCommands();
      if (time_discontinuous) {
        have_cloud_stamp_ = false;
        controller_.reset();
      }
    }
    if (!time_paused && !time_discontinuous) {fsm_time_ += motion_dt;}

    const bool fresh = have_cloud_ &&
      std::chrono::duration<double>(now_tp - last_cloud_time_).count() <= cmd_timeout_;
    // Sensor recovery must not resume motion without an explicit enable.
    if (!fresh) {active_ = false;}
    const bool observed = have_target_observation_ &&
      std::chrono::duration<double>(now_tp - last_target_observation_).count() <=
      target_observation_timeout_;
    if (have_target_observation_ && !observed) {
      if (control_mode_ == 1) {active_ = false;}
      have_target_observation_ = false;
      controller_.clearTarget();
      last_result_ = FollowResult();
    } else if (control_mode_ == 1 && active_ && !observed) {
      active_ = false;
    }
    const bool direct_fresh = have_direct_ &&
      std::chrono::duration<double>(now_tp - last_direct_time_).count() <= direct_cmd_timeout_;
    const bool stop_gate = estop_ || !active_ || !fresh ||
      (control_mode_ == 0 && !direct_fresh);

    geometry_msgs::msg::Twist desired;  // zero unless a valid, fresh command exists
    bool emergency = false;
    if (!stop_gate) {
      if (control_mode_ == 0) {                 // DIRECT (joystick)
        desired.linear.x = direct_vx_;
        desired.linear.y = direct_vy_;
        desired.angular.z = direct_wz_;
      } else if (last_result_.target_valid) {   // FOLLOW
        emergency = last_result_.emergency_stop;
        if (!emergency) {desired = last_result_.cmd;}
        // The tracker has the person: remember where, so a later loss can be
        // searched for instead of ending in a permanent stall.
        search_.noteSighting(fsm_time_, pose_x_, pose_y_);
      } else {                                  // no target: search for them
        search_.noteLost(fsm_time_);
      }

      // ---- P2: target search (only while the tracker has nothing) ----
      if (control_mode_ != 0 && !last_result_.target_valid) {
        double svx = 0.0, swz = 0.0;
        bool arrived = false;
        if (search_.update(fsm_time_, pose_x_, pose_y_, &svx, &swz, &arrived)) {
          desired.linear.x = svx;
          desired.angular.z = swz;
        } else {
          desired = geometry_msgs::msg::Twist();   // HOLD / COAST: stand still
        }
      }

      // ---- P2: stuck recovery ----
      double rvx = 0.0, rvy = 0.0, rwz = 0.0;
      const bool rcmd = (!emergency && control_mode_ != 0 && last_result_.target_valid &&
        recovery_.update(fsm_time_, pose_x_, pose_y_, pose_yaw_,
                         desired.linear.x, desired.linear.y, desired.angular.z,
                         last_result_.clearance_rear, &rvx, &rvy, &rwz));
      if (rcmd) {
        desired.linear.x = rvx;
        desired.linear.y = rvy;
        desired.angular.z = rwz;
        recovery_active_ = true;
      } else {
        recovery_active_ = false;
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

    geometry_msgs::msg::Twist out;
    if (time_paused && !stop_gate && !emergency && finiteTwist(desired)) {
      // No simulation-time advancement: publish zero without evolving the smoother.
    } else if (stop_gate || emergency || !finiteTwist(desired)) {
      resetCommands();
    } else {
      out = smoother_.step(desired, motion_dt, false);
      if (!finiteTwist(out)) {
        out = geometry_msgs::msg::Twist();
        resetCommands();
      }
    }
    // feed the actually-commanded motion back so the target filter can work in
    // an inertial frame (compensates the robot's own rotation)
    controller_.setOdomTwist(out.linear.x, out.linear.y, out.angular.z);
    cmd_pub_->publish(out);

    last_pub_vx_ = out.linear.x;
    last_pub_vy_ = out.linear.y;
    last_pub_wz_ = out.angular.z;
    // A paused, freshly bound target remains selectable for an explicit start.
    // Prediction and cached target messages never extend observation validity.
    const bool target_valid = !estop_ && fresh && observed && controller_.targetValid();
    char control_state_json[96];
    std::snprintf(control_state_json, sizeof(control_state_json),
      "{\"active\":%s,\"mode\":%d,\"estop\":%s,\"target_valid\":%s}",
      active_ ? "true" : "false", control_mode_, estop_ ? "true" : "false",
      target_valid ? "true" : "false");
    std_msgs::msg::String control_state;
    control_state.data = control_state_json;
    control_state_pub_->publish(control_state);

    // ---- diagnostics: which FSM is acting, and how far the robot got ----
    // Machine-readable state, sampled by the acceptance harness. The numbers
    // are included because a frozen robot is otherwise indistinguishable from
    // one whose controller legitimately commanded zero: min_obs tells you what
    // the safety layer actually saw, and vlim why it slowed.
    char buf[320];
    const auto f2 = [](double v) {
        return std::isfinite(v) ? v : -1.0;
      };
    std::snprintf(
      buf, sizeof(buf),
      "recovery=%s attempts=%d search=%s min_obs=%.3f d_stop=%.3f d_slow=%.3f "
      "vlim=%.3f clr_f=%.3f clr_r=%.3f vfh=%.2f vfh_free=%.3f vfh_trav=%d "
      "vfh_block=%d esc=%d excl=%d",
      toString(recovery_.state()), recovery_.attempts(),
      toString(search_.state()),
      f2(last_result_.min_obstacle_dist),
      f2(last_result_.clearance_used),
      f2(last_result_.clearance_slow),
      f2(last_result_.speed_limit),
      f2(last_result_.clearance_front),
      f2(last_result_.clearance_rear),
      last_result_.vfh_dir, f2(last_result_.vfh_free),
      last_result_.vfh_traversable ? 1 : 0, last_result_.vfh_blocked ? 1 : 0,
      recovery_active_ ? 1 : 0, last_result_.target_excluded ? 1 : 0);
    std_msgs::msg::String st;
    st.data = buf;
    state_pub_->publish(st);
  }

  void publishCloudViz(const builtin_interfaces::msg::Time & stamp,
    const std::vector<PointXYZ> & points)
  {
    sensor_msgs::msg::PointCloud2 out;
    out.header.stamp = stamp;
    out.header.frame_id = control_frame_;
    out.height = 1;
    out.width = static_cast<uint32_t>(points.size());
    out.is_bigendian = false;
    out.is_dense = true;
    out.point_step = 12;
    out.row_step = out.width * out.point_step;
    out.fields.resize(3);
    for (size_t axis = 0; axis < 3; ++axis) {
      out.fields[axis].name = axis == 0 ? "x" : (axis == 1 ? "y" : "z");
      out.fields[axis].offset = static_cast<uint32_t>(axis * 4);
      out.fields[axis].datatype = sensor_msgs::msg::PointField::FLOAT32;
      out.fields[axis].count = 1;
    }
    out.data.resize(out.row_step);
    for (size_t index = 0; index < points.size(); ++index) {
      const float values[] = {points[index].x, points[index].y, points[index].z};
      for (size_t axis = 0; axis < 3; ++axis) {
        uint32_t bits;
        std::memcpy(&bits, &values[axis], sizeof(bits));
        for (size_t byte = 0; byte < 4; ++byte) {
          out.data[index * 12 + axis * 4 + byte] =
            static_cast<uint8_t>((bits >> (byte * 8)) & 0xffu);
        }
      }
    }
    cloud_viz_pub_->publish(out);
    last_viz_publish_ = std::chrono::steady_clock::now();
    have_viz_publish_ = true;
    pending_empty_viz_ = false;
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
  }

  // topics
  std::string input_topic_;
  std::string control_frame_;
  std::string cmd_vel_topic_;
  std::string odom_topic_ = "/odom";

  // runtime
  ProjectionConfig proj_cfg_;
  FollowConfig follow_cfg_;
  SmootherConfig smoother_cfg_;
  FollowController controller_;
  RecoveryMachine recovery_;
  TargetSearchMachine search_;
  RecoveryConfig recovery_cfg_;
  SearchConfig search_cfg_;
  double fsm_time_ = 0.0;
  bool recovery_active_ = false;
  double pose_x_ = 0.0, pose_y_ = 0.0, pose_yaw_ = 0.0;
  bool have_pose_ = false;
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
  double direct_cmd_timeout_ = 0.3;
  double target_observation_timeout_ = 0.5;
  bool estop_ = false;
  bool have_cloud_ = false;
  bool have_direct_ = false;
  bool have_target_observation_ = false;
  bool have_cloud_stamp_ = false;
  int64_t last_cloud_stamp_ns_ = 0;
  bool have_viz_publish_ = false;
  bool pending_empty_viz_ = false;
  builtin_interfaces::msg::Time empty_viz_stamp_;
  std::chrono::steady_clock::time_point last_viz_publish_{};
  std::chrono::steady_clock::time_point last_cloud_time_{};
  std::chrono::steady_clock::time_point last_direct_time_{};
  std::chrono::steady_clock::time_point last_target_observation_{};
  std::chrono::steady_clock::time_point last_tick_time_ = std::chrono::steady_clock::now();
  bool have_tick_ros_time_ = false;
  int64_t last_tick_ros_ns_ = 0;
  std::chrono::steady_clock::time_point last_ros_advance_time_ = std::chrono::steady_clock::now();

  // slip / speed compensation
  bool compensate_slip_ = false;
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
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Service<rs_follow_interfaces::srv::BindTarget>::SharedPtr bind_service_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_raw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_viz_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_state_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
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
