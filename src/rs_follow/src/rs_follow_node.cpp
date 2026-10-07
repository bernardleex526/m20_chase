/**
 * @file rs_follow_node.cpp
 * @brief ROS 2 adapter for the rs_follow_core person-following algorithm.
 *
 * This node used to CONTAIN the algorithm: the control law, the safety layer,
 * the planner and the state machines all lived here, entangled with rclcpp. The
 * evaluation report flagged the consequence -- the algorithm could not be used
 * from a non-ROS upper computer, could not be unit-tested without a ROS
 * runtime, and could not be bound to Python.
 *
 * The algorithm now lives in `rs_follow_core` (a plain CMake package with no ROS
 * dependency at all). This file is deliberately thin: it converts messages,
 * owns the ROS timers/subscriptions, and publishes. All the behaviour --
 * dual-band projection, Kalman target filter, VFH+ steering, TTC speed governor,
 * multi-target identity, dynamic obstacle prediction, recovery and search --
 * is in the core and is exercised by the no-ROS test in that package.
 *
 * Topic contract (unchanged from the previous version, so existing harnesses
 * keep working):
 *   in : <input_topic>        sensor_msgs/PointCloud2
 *        <odom_topic>         nav_msgs/Odometry
 *        /clicked_point       geometry_msgs/PointStamped   (RViz "Publish Point")
 *        /rs_follow/bind_target      geometry_msgs/PointStamped
 *        /rs_follow/clear_target     std_msgs/Bool
 *        /rs_follow/enable           std_msgs/Bool
 *        /rs_follow/control_mode     std_msgs/Int32   (0 = DIRECT, 1 = FOLLOW)
 *        /rs_follow/direct_cmd       geometry_msgs/Twist
 *   out: <cmd_vel_topic>      geometry_msgs/Twist
 *        /rs_follow/scan             sensor_msgs/LaserScan
 *        /rs_follow/target           geometry_msgs/PointStamped
 *        /rs_follow/target_raw       geometry_msgs/PointStamped
 *        /rs_follow/status           std_msgs/String
 *        /rs_follow/state            std_msgs/String  (machine-readable)
 *        /rs_follow/target_marker    visualization_msgs/Marker
 *        /rs_follow/tracks           visualization_msgs/MarkerArray (multi-target)
 *
 * Control law adapted from jie_deamon (MIT), https://github.com/6-robot/jie_deamon
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

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
#include <visualization_msgs/msg/marker_array.hpp>

#include "rs_follow/ros_adapter.hpp"
#include "rs_follow_core/lidar_follower.hpp"
#include "rs_follow_core/perf_monitor.hpp"

namespace rs_follow
{

class RsFollowNode : public rclcpp::Node
{
public:
  RsFollowNode()
  : rclcpp::Node("rs_follow_node")
  {
    loadParams();
    follower_.setConfig(cfg_);
    follower_.perf().enable = perf_enable_;
    follower_.perf().report_every = perf_report_every_;
    follower_.setEnabled(cfg_.active);

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
          follower_.clearTarget();
          RCLCPP_INFO(get_logger(), "target cleared");
        }
      });

    enable_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/rs_follow/enable", 10,
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        follower_.setEnabled(msg->data);
        RCLCPP_INFO(get_logger(), "follow %s", msg->data ? "ENABLED" : "DISABLED");
      });

    // direct-control mode (0 = DIRECT joystick, 1 = FOLLOW)
    mode_sub_ = create_subscription<std_msgs::msg::Int32>(
      "/rs_follow/control_mode", 10,
      [this](std_msgs::msg::Int32::SharedPtr msg) {
        follower_.setControlMode(msg->data);
        RCLCPP_INFO(get_logger(), "control_mode -> %s",
                    msg->data == 0 ? "DIRECT" : "FOLLOW");
      });
    direct_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/rs_follow/direct_cmd", 10,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        follower_.setDirectCommand(fromRosTwist(*msg));
      });

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/rs_follow/scan", 10);
    target_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/rs_follow/target", 10);
    target_raw_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/rs_follow/target_raw", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/rs_follow/status", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/rs_follow/target_marker", 10);
    state_pub_ = create_publisher<std_msgs::msg::String>("/rs_follow/state", 10);
    tracks_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/rs_follow/tracks", 10);

    const double period = 1.0 / std::max(1.0, cfg_.control_rate_hz);
    control_timer_ = create_wall_timer(
      std::chrono::duration<double>(period),
      std::bind(&RsFollowNode::controlTimer, this));

    RCLCPP_INFO(
      get_logger(),
      "rs_follow ready | cloud='%s' cmd_vel='%s' auto_front=%s follow_dist=%.2f "
      "multi_target=%s dynamic_obstacle=%s perf=%s",
      input_topic_.c_str(), cmd_vel_topic_.c_str(),
      cfg_.follow.auto_select_front ? "on" : "off", cfg_.follow.follow_dist,
      cfg_.multi_target.enable ? "on" : "off",
      cfg_.dynamic_obstacle.enable ? "on" : "off",
      perf_enable_ ? "on" : "off");
  }

private:
  void loadParams()
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/rslidar_points");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    odom_topic_ = declare_parameter<std::string>("odom_topic", odom_topic_);

    cfg_.compensate_slip = declare_parameter<bool>("compensate_slip", cfg_.compensate_slip);
    cfg_.slip_min_ratio = declare_parameter<double>("slip_min_ratio", cfg_.slip_min_ratio);
    cfg_.slip_max_ratio = declare_parameter<double>("slip_max_ratio", cfg_.slip_max_ratio);
    cfg_.slip_filter_alpha = declare_parameter<double>("slip_filter_alpha", cfg_.slip_filter_alpha);
    cfg_.slip_min_cmd = declare_parameter<double>("slip_min_cmd", cfg_.slip_min_cmd);
    cfg_.max_linear_cmd = declare_parameter<double>("max_linear_cmd", cfg_.max_linear_cmd);
    cfg_.max_angular_cmd = declare_parameter<double>("max_angular_cmd", cfg_.max_angular_cmd);
    cfg_.control_rate_hz = declare_parameter<double>("control_rate_hz", cfg_.control_rate_hz);
    cfg_.cmd_timeout = declare_parameter<double>("cmd_timeout", cfg_.cmd_timeout);
    cfg_.active = declare_parameter<bool>("active", cfg_.active);
    publish_scan_debug_ = declare_parameter<bool>("publish_scan_debug", true);

    // --- 3D cloud -> 2D polar scan projection (TARGET band: person torso) ---
    cfg_.projection.height_min =
      declare_parameter<double>("height_min", cfg_.projection.height_min);
    cfg_.projection.height_max =
      declare_parameter<double>("height_max", cfg_.projection.height_max);
    cfg_.projection.z_offset =
      declare_parameter<double>("z_offset", cfg_.projection.z_offset);
    cfg_.projection.angle_bins =
      declare_parameter<int>("angle_bins", cfg_.projection.angle_bins);
    cfg_.projection.range_min =
      declare_parameter<double>("range_min", cfg_.projection.range_min);
    cfg_.projection.range_max =
      declare_parameter<double>("range_max", cfg_.projection.range_max);
    cfg_.projection.flip_x = declare_parameter<bool>("flip_x", cfg_.projection.flip_x);
    cfg_.projection.flip_y = declare_parameter<bool>("flip_y", cfg_.projection.flip_y);
    cfg_.projection.enable_low_band =
      declare_parameter<bool>("enable_low_band", cfg_.projection.enable_low_band);
    cfg_.projection.low_height_min =
      declare_parameter<double>("low_height_min", cfg_.projection.low_height_min);
    cfg_.projection.low_height_max =
      declare_parameter<double>("low_height_max", cfg_.projection.low_height_max);
    cfg_.auto_low_band = declare_parameter<bool>("auto_low_band", cfg_.auto_low_band);
    cfg_.sensor_height = declare_parameter<double>("sensor_height", cfg_.sensor_height);
    cfg_.ground_clearance =
      declare_parameter<double>("ground_clearance", cfg_.ground_clearance);
    cfg_.band_separation =
      declare_parameter<double>("band_separation", cfg_.band_separation);

    // --- follow target / standoff ---
    auto & f = cfg_.follow;
    f.follow_dist = declare_parameter<double>("follow_dist", f.follow_dist);
    f.target_radius = declare_parameter<double>("target_radius", f.target_radius);
    f.rectangle_width = declare_parameter<double>("rectangle_width", f.rectangle_width);

    // --- control gains ---
    f.k_linear = declare_parameter<double>("k_linear", f.k_linear);
    f.k_angular = declare_parameter<double>("k_angular", f.k_angular);
    f.k_lateral = declare_parameter<double>("k_lateral", f.k_lateral);
    f.enable_lateral = declare_parameter<bool>("enable_lateral", f.enable_lateral);
    f.rotate_in_place_behind =
      declare_parameter<bool>("rotate_in_place_behind", f.rotate_in_place_behind);

    // --- limits / deadbands ---
    f.max_linear = declare_parameter<double>("max_linear", f.max_linear);
    f.max_angular = declare_parameter<double>("max_angular", f.max_angular);
    f.linear_deadband = declare_parameter<double>("linear_deadband", f.linear_deadband);
    f.angular_deadband = declare_parameter<double>("angular_deadband", f.angular_deadband);
    f.min_linear_speed = declare_parameter<double>("min_linear_speed", f.min_linear_speed);
    f.linear_hysteresis =
      declare_parameter<double>("linear_hysteresis", f.linear_hysteresis);
    f.angular_hysteresis =
      declare_parameter<double>("angular_hysteresis", f.angular_hysteresis);
    f.k_integral = declare_parameter<double>("k_integral", f.k_integral);
    f.integral_limit = declare_parameter<double>("integral_limit", f.integral_limit);

    // --- obstacle handling / safety ---
    f.exclude_target_from_obstacles =
      declare_parameter<bool>("exclude_target_from_obstacles",
                              f.exclude_target_from_obstacles);
    f.target_exclude_slack =
      declare_parameter<double>("target_exclude_slack", f.target_exclude_slack);
    f.direction_gated_obstacles =
      declare_parameter<bool>("direction_gated_obstacles", f.direction_gated_obstacles);
    f.sector_half_angle_deg =
      declare_parameter<double>("sector_half_angle_deg", f.sector_half_angle_deg);
    f.robot_length = declare_parameter<double>("robot_length", f.robot_length);
    f.robot_width = declare_parameter<double>("robot_width", f.robot_width);
    f.auto_frame = declare_parameter<bool>("auto_frame", f.auto_frame);
    f.self_occlusion_margin =
      declare_parameter<double>("self_occlusion_margin", f.self_occlusion_margin);
    f.behind_rotate_timeout =
      declare_parameter<double>("behind_rotate_timeout", f.behind_rotate_timeout);

    // --- legacy APF (superseded by the governor, still honoured) ---
    f.apf_influence = declare_parameter<double>("apf_influence", f.apf_influence);
    f.apf_gain = declare_parameter<double>("apf_gain", f.apf_gain);
    f.apf_emergency = declare_parameter<double>("apf_emergency", f.apf_emergency);
    f.apf_slowdown = declare_parameter<double>("apf_slowdown", f.apf_slowdown);

    // --- explicit self-occlusion box (used when auto_frame is false) ---
    f.frame_front = declare_parameter<double>("frame_front", f.frame_front);
    f.frame_back = declare_parameter<double>("frame_back", f.frame_back);
    f.frame_left = declare_parameter<double>("frame_left", f.frame_left);
    f.frame_right = declare_parameter<double>("frame_right", f.frame_right);

    // --- target filtering / loss ---
    f.enable_kalman = declare_parameter<bool>("enable_kalman", f.enable_kalman);
    f.kalman_q = declare_parameter<double>("kalman_q", f.kalman_q);
    f.kalman_r = declare_parameter<double>("kalman_r", f.kalman_r);
    f.kalman_gate = declare_parameter<double>("kalman_gate", f.kalman_gate);
    f.kalman_r_ref_points =
      declare_parameter<int>("kalman_r_ref_points", f.kalman_r_ref_points);
    f.filter_in_world = declare_parameter<bool>("filter_in_world", f.filter_in_world);
    f.lost_frames_timeout =
      declare_parameter<int>("lost_frames_timeout", f.lost_frames_timeout);

    // --- automatic front target selection ---
    f.auto_select_front = declare_parameter<bool>("auto_select_front", f.auto_select_front);
    f.auto_front_fov_deg =
      declare_parameter<double>("auto_front_fov_deg", f.auto_front_fov_deg);
    f.auto_select_max_range =
      declare_parameter<double>("auto_select_max_range", f.auto_select_max_range);
    f.auto_max_target_width =
      declare_parameter<double>("auto_max_target_width", f.auto_max_target_width);
    f.bind_radius = declare_parameter<double>("bind_radius", f.bind_radius);

    // --- TTC speed governor ---
    auto & g = f.governor;
    g.enable = declare_parameter<bool>("governor_enable", g.enable);
    g.d_hard = declare_parameter<double>("governor_d_hard", g.d_hard);
    g.d_margin = declare_parameter<double>("governor_d_margin", g.d_margin);
    g.t_lat = declare_parameter<double>("governor_t_lat", g.t_lat);
    g.a_max = declare_parameter<double>("governor_a_max", g.a_max);
    g.v_cap = declare_parameter<double>("governor_v_cap", g.v_cap);
    g.release_hysteresis =
      declare_parameter<double>("governor_release_hysteresis", g.release_hysteresis);
    g.d_margin_slow =
      declare_parameter<double>("governor_d_margin_slow", g.d_margin_slow);
    g.emergency_floor =
      declare_parameter<double>("governor_emergency_floor", g.emergency_floor);
    g.lateral_margin =
      declare_parameter<double>("governor_lateral_margin", g.lateral_margin);
    g.stop_margin = declare_parameter<double>("governor_stop_margin", g.stop_margin);

    // --- gap-based local steering (VFH+) ---
    auto & v = f.vfh;
    v.enable = declare_parameter<bool>("vfh_enable", v.enable);
    v.bins = declare_parameter<int>("vfh_bins", v.bins);
    v.d_safe = declare_parameter<double>("vfh_d_safe", v.d_safe);
    v.auto_d_safe = declare_parameter<bool>("vfh_auto_d_safe", v.auto_d_safe);
    v.d_safe_margin = declare_parameter<double>("vfh_d_safe_margin", v.d_safe_margin);
    v.goal_weight = declare_parameter<double>("vfh_goal_weight", v.goal_weight);
    v.hysteresis_deg =
      declare_parameter<double>("vfh_hysteresis_deg", v.hysteresis_deg);
    v.max_turn_deg = declare_parameter<double>("vfh_max_turn_deg", v.max_turn_deg);

    // --- output smoothing ---
    auto & s = cfg_.smoother;
    s.max_linear_accel =
      declare_parameter<double>("max_linear_accel", s.max_linear_accel);
    s.max_angular_accel =
      declare_parameter<double>("max_angular_accel", s.max_angular_accel);
    s.cmd_filter_alpha =
      declare_parameter<double>("cmd_filter_alpha", s.cmd_filter_alpha);
    s.max_linear_jerk = declare_parameter<double>("max_linear_jerk", s.max_linear_jerk);
    s.max_angular_jerk =
      declare_parameter<double>("max_angular_jerk", s.max_angular_jerk);

    // --- recovery / target search ---
    auto & r = cfg_.recovery;
    r.enable = declare_parameter<bool>("recovery_enable", r.enable);
    r.stuck_window = declare_parameter<double>("recovery_stuck_window", r.stuck_window);
    r.stuck_dist = declare_parameter<double>("recovery_stuck_dist", r.stuck_dist);
    r.backup_dist = declare_parameter<double>("recovery_backup_dist", r.backup_dist);
    r.backup_speed = declare_parameter<double>("recovery_backup_speed", r.backup_speed);
    r.spin_rate = declare_parameter<double>("recovery_spin_rate", r.spin_rate);
    r.spin_timeout = declare_parameter<double>("recovery_spin_timeout", r.spin_timeout);
    r.max_attempts =
      declare_parameter<int>("recovery_max_attempts", r.max_attempts);

    auto & se = cfg_.search;
    se.enable = declare_parameter<bool>("search_enable", se.enable);
    se.coast_time = declare_parameter<double>("search_coast_time", se.coast_time);
    se.spin_timeout = declare_parameter<double>("search_spin_timeout", se.spin_timeout);
    se.go_speed = declare_parameter<double>("search_go_speed", se.go_speed);
    se.go_tol = declare_parameter<double>("search_go_tol", se.go_tol);
    se.go_timeout = declare_parameter<double>("search_go_timeout", se.go_timeout);
    se.search_budget = declare_parameter<double>("search_budget", se.search_budget);
    se.max_sweeps = declare_parameter<int>("search_max_sweeps", se.max_sweeps);

    // --- P1 multi-target tracking ---
    auto & mt = cfg_.multi_target;
    mt.enable = declare_parameter<bool>("multi_target_enable", mt.enable);
    mt.max_range = declare_parameter<double>("multi_target_max_range", mt.max_range);
    mt.assoc_gate = declare_parameter<double>("multi_target_assoc_gate", mt.assoc_gate);
    mt.max_missed = declare_parameter<int>("multi_target_max_missed", mt.max_missed);
    mt.max_tracks = declare_parameter<int>("multi_target_max_tracks", mt.max_tracks);
    mt.person_width = declare_parameter<double>("multi_target_person_width", mt.person_width);
    mt.max_width = declare_parameter<double>("multi_target_max_width", mt.max_width);
    const int sel = declare_parameter<int>("multi_target_select", 2);
    mt.select = static_cast<MultiTargetConfig::Select>(std::clamp(sel, 0, 3));

    // --- P1 dynamic obstacle prediction ---
    auto & dyn = cfg_.dynamic_obstacle;
    dyn.enable = declare_parameter<bool>("dynamic_obstacle_enable", dyn.enable);
    dyn.lookahead =
      declare_parameter<double>("dynamic_obstacle_lookahead", dyn.lookahead);
    dyn.corridor_margin =
      declare_parameter<double>("dynamic_obstacle_corridor_margin", dyn.corridor_margin);
    dyn.assoc_gate =
      declare_parameter<double>("dynamic_obstacle_assoc_gate", dyn.assoc_gate);
    dyn.max_range = declare_parameter<double>("dynamic_obstacle_max_range", dyn.max_range);
    dyn.vel_alpha =
      declare_parameter<double>("dynamic_obstacle_vel_alpha", dyn.vel_alpha);

    // --- performance instrumentation ---
    perf_enable_ = declare_parameter<bool>("perf_enable", perf_enable_);
    perf_report_every_ =
      declare_parameter<int>("perf_report_every", perf_report_every_);
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    PointCloud cloud;
    if (!toPointCloud(*msg, cloud)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "cannot convert cloud (missing x/y/z FLOAT32 fields?) topic=%s",
        input_topic_.c_str());
      return;
    }
    if (!follower_.setPointCloud(cloud)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "cannot project cloud topic=%s", input_topic_.c_str());
      return;
    }

    if (publish_scan_debug_) {
      publishScan(follower_.lastScan());
    }
    publishTarget(follower_.lastScan());
    publishTracks(follower_.lastScan());
    publishStatus();

    const auto & r = follower_.lastResult();
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "target=%s range=%.2f bearing=%.1fdeg pts=%d min_obs=%.2f "
      "clr(f/r/u)=%.2f/%.2f/%.2f vlim=%.2f excl=%s cmd=(%.2f,%.2f,%.2f)",
      r.target_valid ? "LOCK" : "NONE",
      r.target_range, r.target_bearing * 180.0 / M_PI,
      r.points_in_target,
      std::isfinite(r.min_obstacle_dist) ? r.min_obstacle_dist : -1.0,
      std::isfinite(r.clearance_front) ? r.clearance_front : -1.0,
      std::isfinite(r.clearance_rear) ? r.clearance_rear : -1.0,
      std::isfinite(r.clearance_used) ? r.clearance_used : -1.0,
      std::isfinite(r.speed_limit) ? r.speed_limit : -1.0,
      r.target_excluded ? "Y" : "n",
      r.cmd.linear_x, r.cmd.linear_y, r.cmd.angular_z);
  }

  void clickedCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    const bool snapped = follower_.bindTarget(msg->point.x, msg->point.y);
    RCLCPP_INFO(
      get_logger(), "bind target (%.2f, %.2f) snapped=%s -> (%.2f, %.2f)",
      msg->point.x, msg->point.y, snapped ? "yes" : "no",
      follower_.targetX(), follower_.targetY());
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    Odometry odom;
    toOdometry(*msg, odom);
    follower_.setOdometry(odom);
    // The measured body twist is what slip compensation and the obstacle
    // velocity estimator need; feeding it separately keeps the two concerns
    // (pose for the inertial filter, twist for slip) independent.
    follower_.setRobotTwist(msg->twist.twist.linear.x,
                            msg->twist.twist.linear.y,
                            msg->twist.twist.angular.z);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "slip comp: lin_ratio=%.2f ang_ratio=%.2f (actual/cmd)",
      follower_.status().slip_lin_ratio, follower_.status().slip_ang_ratio);
  }

  void controlTimer()
  {
    const auto now_tp = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(now_tp - last_tick_time_).count();
    last_tick_time_ = now_tp;

    const Twist cmd = follower_.update(dt);
    geometry_msgs::msg::Twist out;
    toRosTwist(cmd, out);
    {
      PerfScope scope(follower_.perf(), Stage::kPublish);
      cmd_pub_->publish(out);
    }

    // Per-stage timing / memory, when perf_enable is set. The core produces the
    // text (it must stay logger-free to remain usable off-ROS); this node decides
    // where it goes.
    const std::string perf = follower_.perfReport();
    if (!perf.empty()) {
      RCLCPP_INFO(get_logger(), "\n%s", perf.c_str());
    }

    const FollowerStatus & st = follower_.status();
    const FollowResult & r = st.follow;
    char buf[420];
    const auto f2 = [](double v) {
        return std::isfinite(v) ? v : -1.0;
      };
    std::snprintf(
      buf, sizeof(buf),
      "recovery=%s attempts=%d search=%s min_obs=%.3f d_stop=%.3f d_slow=%.3f "
      "vlim=%.3f clr_f=%.3f clr_r=%.3f vfh=%.2f vfh_free=%.3f vfh_trav=%d "
      "vfh_block=%d esc=%d excl=%d tracks=%d obs_tracks=%d tid=%d "
      "cross=%.3f cross_cls=%.3f",
      toString(st.recovery_state), st.recovery_attempts,
      toString(st.search_state),
      f2(r.min_obstacle_dist),
      f2(r.clearance_used),
      f2(r.clearance_slow),
      f2(r.speed_limit),
      f2(r.clearance_front),
      f2(r.clearance_rear),
      r.vfh_dir, f2(r.vfh_free),
      r.vfh_traversable ? 1 : 0, r.vfh_blocked ? 1 : 0,
      st.recovery_active ? 1 : 0, r.target_excluded ? 1 : 0,
      st.track_count, st.obstacle_track_count, st.target_track_id,
      f2(st.crossing_clearance), st.crossing_closing);
    std_msgs::msg::String smsg;
    smsg.data = buf;
    state_pub_->publish(smsg);
  }

  void publishScan(const ScanFrame & scan)
  {
    sensor_msgs::msg::LaserScan out;
    out.header.stamp = now();
    out.header.frame_id = scan.frame_id;
    out.angle_min = static_cast<float>(scan.angle_min);
    out.angle_max = static_cast<float>(
      scan.angle_min + scan.angle_increment * (scan.ranges.size() - 1));
    out.angle_increment = static_cast<float>(scan.angle_increment);
    out.range_min = static_cast<float>(cfg_.projection.range_min);
    out.range_max = static_cast<float>(cfg_.projection.range_max);
    out.ranges = scan.ranges;
    scan_pub_->publish(out);
  }

  void publishTarget(const ScanFrame & scan)
  {
    const FollowResult & r = follower_.lastResult();
    if (!r.target_valid) {
      return;
    }
    geometry_msgs::msg::PointStamped pt;
    pt.header.stamp = now();
    pt.header.frame_id = scan.frame_id;
    pt.point.x = r.target_x;
    pt.point.y = r.target_y;
    pt.point.z = 0.0;
    target_pub_->publish(pt);

    geometry_msgs::msg::PointStamped raw;
    raw.header = pt.header;
    raw.point.x = r.target_raw_x;
    raw.point.y = r.target_raw_y;
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
    m.color.r = r.target_manual ? 0.1f : 0.1f;
    m.color.g = r.target_manual ? 0.9f : 0.5f;
    m.color.b = r.target_manual ? 0.1f : 0.9f;
    marker_pub_->publish(m);
  }

  /** @brief Publish every multi-target track, so identity is visible in RViz. */
  void publishTracks(const ScanFrame & scan)
  {
    if (!cfg_.multi_target.enable) {
      return;
    }
    const auto & tracks = follower_.multiTarget().tracks();
    const int locked = follower_.multiTarget().lockedId();
    visualization_msgs::msg::MarkerArray arr;
    // clear stale markers from a previous, larger set
    {
      visualization_msgs::msg::Marker del;
      del.header.stamp = now();
      del.header.frame_id = scan.frame_id;
      del.ns = "rs_follow_tracks";
      del.action = visualization_msgs::msg::Marker::DELETEALL;
      arr.markers.push_back(del);
    }
    for (size_t i = 0; i < tracks.size(); ++i) {
      const auto & t = tracks[i];
      visualization_msgs::msg::Marker m;
      m.header.stamp = now();
      m.header.frame_id = scan.frame_id;
      m.ns = "rs_follow_tracks";
      m.id = t.id + 1;              // 0 is reserved for the DELETEALL marker
      m.type = visualization_msgs::msg::Marker::CYLINDER;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = t.x;
      m.pose.position.y = t.y;
      m.pose.position.z = 0.5;
      m.pose.orientation.w = 1.0;
      m.scale.x = m.scale.y = std::max(0.2, t.width_m);
      m.scale.z = 1.0;
      m.color.a = 0.5f;
      const bool is_locked = (t.id == locked);
      m.color.r = is_locked ? 0.1f : 0.6f;
      m.color.g = is_locked ? 0.9f : 0.6f;
      m.color.b = is_locked ? 0.1f : 0.6f;
      arr.markers.push_back(m);
    }
    tracks_pub_->publish(arr);
  }

  void publishStatus()
  {
    std_msgs::msg::String s;
    s.data = follower_.status().statusText();
    status_pub_->publish(s);
  }

  // topics
  std::string input_topic_;
  std::string cmd_vel_topic_;
  std::string odom_topic_ = "/odom";
  bool publish_scan_debug_ = true;
  bool perf_enable_ = false;
  int perf_report_every_ = 0;

  // the algorithm
  FollowerConfig cfg_;
  LidarFollower follower_;

  std::chrono::steady_clock::time_point last_tick_time_ = std::chrono::steady_clock::now();

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
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tracks_pub_;
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
