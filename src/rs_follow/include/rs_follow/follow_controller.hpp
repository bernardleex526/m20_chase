/**
 * @file follow_controller.hpp
 * @brief Target binding / tracking and follow velocity law for a 2D polar scan.
 *
 * Behaviour adapted from jie_deamon LidarTracker (MIT License),
 * https://github.com/6-robot/jie_deamon :
 *   distance error -> linear.x, bearing -> angular.z,
 *   corridor (left/right wall) -> linear.y, plus artificial potential field
 *   obstacle repulsion and emergency stop.
 */

#ifndef RS_FOLLOW_FOLLOW_CONTROLLER_HPP
#define RS_FOLLOW_FOLLOW_CONTROLLER_HPP

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>

#include <geometry_msgs/msg/twist.hpp>

#include "rs_follow/kalman_filter_2d.hpp"
#include "rs_follow/pointcloud_scan.hpp"

namespace rs_follow
{

struct FollowConfig
{
  // desired standoff distance and cluster radius
  double follow_dist = 1.0;
  double target_radius = 0.6;
  double rectangle_width = 0.5;

  // control gains
  double k_linear = 2.5;
  double k_angular = 1.5;
  double k_lateral = 0.8;
  bool enable_lateral = true;
  bool rotate_in_place_behind = true;  // no forward motion if target > 90 deg off

  // speed limits
  double max_linear = 0.9;
  double max_angular = 1.0;

  // deadbands / minimum speed
  double linear_deadband = 0.05;
  double angular_deadband = 0.08;
  double min_linear_speed = 0.02;
  // hysteresis added on top of the deadband to stop on/off chatter
  double linear_hysteresis = 0.03;
  double angular_hysteresis = 0.03;
  // integral action (removes steady-state lag when the target moves)
  double k_integral = 0.3;
  double integral_limit = 1.0;   // clamp on integral of distance error (m*s)

  // artificial potential field
  double apf_influence = 0.6;
  double apf_gain = 0.02;
  double apf_emergency = 0.35;
  double apf_slowdown = 0.7;

  // robot self-occlusion box in lidar frame (x forward, y left)
  double frame_front = 0.25;
  double frame_back = 0.45;
  double frame_left = 0.25;
  double frame_right = 0.25;

  // target filtering / loss handling
  bool enable_kalman = true;
  double kalman_q = 0.1;
  double kalman_r = 0.05;
  double kalman_gate = 6.0;        // Mahalanobis gate (sigma); 0 disables
  int kalman_r_ref_points = 80;    // point count for nominal R; scales as n_ref/n
  // Filter the target in an inertial frame (compensates the robot's own rotation).
  // Requires feeding the robot's actual motion via setOdomTwist().
  bool filter_in_world = true;
  int lost_frames_timeout = 30;

  // automatic target selection when nothing is bound
  bool auto_select_front = true;
  double auto_front_fov_deg = 40.0;
  double auto_select_max_range = 8.0;

  // manual binding
  double bind_radius = 0.6;
};

struct FollowResult
{
  geometry_msgs::msg::Twist cmd;
  bool target_valid = false;
  bool target_observed = false;
  bool target_manual = false;
  bool emergency_stop = false;
  double target_range = 0.0;     // metres, lidar frame
  double target_bearing = 0.0;   // radians, 0 = straight ahead, + = left
  double target_x = 0.0;
  double target_y = 0.0;
  double target_raw_x = 0.0;
  double target_raw_y = 0.0;
  double min_obstacle_dist = std::numeric_limits<double>::infinity();
  int points_in_target = 0;
};

class FollowController
{
public:
  explicit FollowController(const FollowConfig & cfg = FollowConfig())
  : cfg_(cfg)
  {
    applyKalmanConfig();
  }

  void setConfig(const FollowConfig & cfg)
  {
    cfg_ = cfg;
    applyKalmanConfig();
  }
  const FollowConfig & config() const {return cfg_;}

  /**
   * @brief Feed the robot's actual motion (odometry / published cmd_vel).
   * Used to keep the target filter in an inertial frame (see filter_in_world).
   */
  void setOdomTwist(double vx, double vy, double wz)
  {
    odom_vx_ = vx;
    odom_vy_ = vy;
    odom_wz_ = wz;
  }

  /**
   * @brief Feed the robot's measured pose (odometry / SLAM). When set, the
   * inertial target filter uses this pose instead of integrating cmd_vel, which
   * removes drift caused by slip / tracking error.
   */
  void setOdomPose(double x, double y, double yaw)
  {
    if (!has_odom_pose_) {
      has_odom_pose_ = true;
      if (cfg_.filter_in_world && target_valid_) {
        dead_x_ = x;
        dead_y_ = y;
        dead_yaw_ = yaw;
        double wx, wy;
        sensorToWorld(target_x_, target_y_, wx, wy);
        kalman_.setState(wx, wy);
      }
    }
    dead_x_ = x;
    dead_y_ = y;
    dead_yaw_ = yaw;
  }

  void reset()
  {
    target_valid_ = false;
    target_manual_ = false;
    lost_frames_ = 0;
    linear_dir_ = 0;
    angular_dir_ = 0;
    linear_integral_ = 0.0;
    kalman_.reset();
    target_x_ = cfg_.follow_dist;
    target_y_ = 0.0;
  }

  void clearTarget()
  {
    target_valid_ = false;
    target_manual_ = false;
    lost_frames_ = 0;
    linear_dir_ = 0;
    angular_dir_ = 0;
    linear_integral_ = 0.0;
    kalman_.reset();
  }

  bool targetValid() const {return target_valid_;}
  bool targetManual() const {return target_manual_;}
  double targetX() const {return target_x_;}
  double targetY() const {return target_y_;}

  /**
   * @brief Bind a target from a clicked/selected point (lidar frame).
   * Snaps to the nearest scan return within bind_radius when possible.
   */
  bool bindTarget(double x, double y, const ScanFrame & scan)
  {
    double best_x = x, best_y = y;
    double best_d2 = cfg_.bind_radius * cfg_.bind_radius;
    bool snapped = false;

    for (int i = 0; i < static_cast<int>(scan.ranges.size()); ++i) {
      if (!std::isfinite(scan.ranges[static_cast<size_t>(i)])) {
        continue;
      }
      const double a = scan.angleAt(i);
      const double r = scan.ranges[static_cast<size_t>(i)];
      const double px = r * std::cos(a);
      const double py = r * std::sin(a);
      const double d2 = (px - x) * (px - x) + (py - y) * (py - y);
      if (d2 < best_d2) {
        best_d2 = d2;
        best_x = px;
        best_y = py;
        snapped = true;
      }
    }

    target_x_ = best_x;
    target_y_ = best_y;
    target_valid_ = true;
    target_manual_ = true;
    lost_frames_ = 0;
    linear_dir_ = 0;
    angular_dir_ = 0;
    linear_integral_ = 0.0;
    {
      double sx = best_x, sy = best_y;
      if (cfg_.filter_in_world) {
        sensorToWorld(best_x, best_y, sx, sy);
      }
      kalman_.setState(sx, sy);
    }
    last_obs_time_ = std::chrono::steady_clock::now();
    return snapped;
  }

  /**
   * @brief Update tracking + control from the latest scan.
   */
  FollowResult update(const ScanFrame & scan)
  {
    FollowResult res;

    // control period (for integral action)
    auto now_tp = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now_tp - last_control_time_).count();
    last_control_time_ = now_tp;
    dt = std::clamp(dt, 0.02, 0.5);

    if (cfg_.filter_in_world && !has_odom_pose_) {
      integrateOdom(dt);
    }

    // ---- target acquisition ----
    if (!target_valid_ && cfg_.auto_select_front) {
      autoSelectFront(scan);
    }
    if (!target_valid_) {
      res.target_valid = false;
      return res;
    }

    // ---- collect points, corridor, obstacle field in one pass ----
    const double half_w = cfg_.rectangle_width / 2.0;
    double centroid_x = 0.0, centroid_y = 0.0;
    int in_target = 0;

    double repulse_x = 0.0, repulse_y = 0.0;
    double min_obstacle = std::numeric_limits<double>::infinity();

    const double tvx = target_x_;
    const double tvy = target_y_;
    const double tv_len = std::hypot(tvx, tvy);

    double left_y_min = -half_w;
    double right_y_min = half_w;

    for (int i = 0; i < static_cast<int>(scan.ranges.size()); ++i) {
      const float rf = scan.ranges[static_cast<size_t>(i)];
      if (!std::isfinite(rf)) {
        continue;
      }
      const double a = scan.angleAt(i);
      const double px = rf * std::cos(a);
      const double py = rf * std::sin(a);
      const double dist = rf;

      const bool in_frame =
        (px > -cfg_.frame_back && px < cfg_.frame_front &&
        py > -cfg_.frame_right && py < cfg_.frame_left);

      if (!in_frame && dist < min_obstacle) {
        min_obstacle = dist;
      }
      if (!in_frame && dist < cfg_.apf_influence && px > -0.1) {
        const double force = cfg_.apf_gain *
          (1.0 / dist - 1.0 / cfg_.apf_influence) / (dist * dist);
        repulse_x -= force * px / dist;
        repulse_y -= force * py / dist;
      }
      if (in_frame) {
        continue;
      }

      const double d2t = (px - target_x_) * (px - target_x_) +
        (py - target_y_) * (py - target_y_);
      if (d2t < cfg_.target_radius * cfg_.target_radius) {
        centroid_x += px;
        centroid_y += py;
        ++in_target;
        continue;
      }

      if (tv_len > 1e-6) {
        const double proj_x = (px * tvx + py * tvy) / tv_len;
        const double proj_y = (px * -tvy + py * tvx) / tv_len;
        if (proj_x >= 0.0 && proj_x <= tv_len && std::abs(proj_y) <= half_w) {
          if (proj_y > 0.0 && proj_y > left_y_min) {
            left_y_min = proj_y;
          } else if (proj_y <= 0.0 && proj_y < right_y_min) {
            right_y_min = proj_y;
          }
        }
      }
    }

    // ---- target update ----
    res.points_in_target = in_target;
    bool measurement_ok = false;
    if (in_target > 0) {
      const double raw_x = centroid_x / in_target;
      const double raw_y = centroid_y / in_target;
      res.target_raw_x = raw_x;
      res.target_raw_y = raw_y;
      measurement_ok = true;
      if (cfg_.enable_kalman) {
        double fx, fy;
        bool accepted = true;
        if (cfg_.filter_in_world) {
          double mx, my;
          sensorToWorld(raw_x, raw_y, mx, my);
          kalman_.update(mx, my, fx, fy, &accepted, in_target);
          worldToSensor(fx, fy, target_x_, target_y_);
        } else {
          kalman_.update(raw_x, raw_y, fx, fy, &accepted, in_target);
          target_x_ = fx;
          target_y_ = fy;
        }
        measurement_ok = accepted;  // outlier rejected -> treat as not observed
      } else {
        target_x_ = raw_x;
        target_y_ = raw_y;
      }
    }
    res.target_observed = measurement_ok;

    if (measurement_ok) {
      lost_frames_ = 0;
      last_obs_time_ = std::chrono::steady_clock::now();
    } else {
      ++lost_frames_;
      if (lost_frames_ > cfg_.lost_frames_timeout) {
        clearTarget();
        res.target_valid = false;
        return res;
      }
      if (cfg_.enable_kalman) {
        double dt_lost = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - last_obs_time_).count();
        dt_lost = std::clamp(dt_lost, 0.02, 0.5);
        double fx, fy;
        kalman_.predictOnly(dt_lost, fx, fy);
        if (cfg_.filter_in_world) {
          worldToSensor(fx, fy, target_x_, target_y_);
        } else {
          target_x_ = fx;
          target_y_ = fy;
        }
      }
    }

    // ---- control law ----
    const double range = std::hypot(target_x_, target_y_);
    const double bearing = std::atan2(target_y_, target_x_);
    res.target_range = range;
    res.target_bearing = bearing;
    res.target_x = target_x_;
    res.target_y = target_y_;
    res.target_valid = true;
    res.target_manual = target_manual_;
    res.min_obstacle_dist = min_obstacle;

    geometry_msgs::msg::Twist cmd;

    if (min_obstacle < cfg_.apf_emergency) {
      res.emergency_stop = true;
      res.cmd = cmd;  // all zero
      return res;
    }

    // forward / backward by distance error (Euclidean range), with hysteresis
    const double dist_error = range - cfg_.follow_dist;
    const bool target_behind = std::abs(bearing) > (M_PI / 2.0);
    const double ad = std::abs(dist_error);
    if (linear_dir_ == 0) {
      if (ad >= cfg_.linear_deadband + cfg_.linear_hysteresis) {
        linear_dir_ = (dist_error > 0.0) ? 1 : -1;
      }
    } else if (ad <= cfg_.linear_deadband) {
      linear_dir_ = 0;
    }
    if (linear_dir_ == 0 || (cfg_.rotate_in_place_behind && target_behind)) {
      cmd.linear.x = 0.0;
    } else {
      cmd.linear.x = dist_error * cfg_.k_linear;
      if (cmd.linear.x < 0.0) {
        cmd.linear.x *= 0.8;  // back off more gently
      }
      if (std::abs(cmd.linear.x) < cfg_.min_linear_speed) {
        cmd.linear.x = (cmd.linear.x > 0.0) ? cfg_.min_linear_speed : -cfg_.min_linear_speed;
      }
    }

    // integral action: removes the proportional-only steady-state lag on a moving target
    const bool integrating = (linear_dir_ != 0) &&
      !(cfg_.rotate_in_place_behind && target_behind) && std::abs(dist_error) < 1.5;
    if (integrating) {
      linear_integral_ += dist_error * dt;
      linear_integral_ = std::clamp(linear_integral_, -cfg_.integral_limit, cfg_.integral_limit);
      cmd.linear.x += cfg_.k_integral * linear_integral_;
    } else if (linear_dir_ == 0) {
      linear_integral_ *= 0.95;  // bleed off when inside the deadband
    }

    // rotation by bearing, with hysteresis
    const double ab = std::abs(bearing);
    if (angular_dir_ == 0) {
      if (ab >= cfg_.angular_deadband + cfg_.angular_hysteresis) {
        angular_dir_ = (bearing > 0.0) ? 1 : -1;
      }
    } else if (ab <= cfg_.angular_deadband) {
      angular_dir_ = 0;
    }
    if (angular_dir_ == 0) {
      cmd.angular.z = 0.0;
    } else {
      cmd.angular.z = bearing * cfg_.k_angular;
    }

    // lateral centring inside a corridor / behind the target
    if (cfg_.enable_lateral) {
      double lateral_error = -(left_y_min + right_y_min);
      if (std::abs(lateral_error) > 1.0) {
        lateral_error = 0.0;
      }
      if (std::abs(lateral_error) < 0.03) {
        cmd.linear.y = 0.0;
      } else {
        cmd.linear.y = lateral_error * cfg_.k_lateral;
      }
    }

    // potential field repulsion
    cmd.linear.x += repulse_x;
    cmd.linear.y += repulse_y;

    // slow down near obstacles
    if (min_obstacle < cfg_.apf_slowdown) {
      double f = (min_obstacle - cfg_.apf_emergency) /
        (cfg_.apf_slowdown - cfg_.apf_emergency);
      f = std::clamp(f, 0.1, 1.0);
      cmd.linear.x *= f;
    }

    cmd.linear.x = std::clamp(cmd.linear.x, -cfg_.max_linear, cfg_.max_linear);
    cmd.linear.y = std::clamp(cmd.linear.y, -cfg_.max_linear, cfg_.max_linear);
    cmd.angular.z = std::clamp(cmd.angular.z, -cfg_.max_angular, cfg_.max_angular);

    res.cmd = cmd;
    return res;
  }

private:
  void integrateOdom(double dt)
  {
    dead_yaw_ += odom_wz_ * dt;
    const double c = std::cos(dead_yaw_), s = std::sin(dead_yaw_);
    dead_x_ += (odom_vx_ * c - odom_vy_ * s) * dt;
    dead_y_ += (odom_vx_ * s + odom_vy_ * c) * dt;
  }

  void sensorToWorld(double sx, double sy, double & wx, double & wy) const
  {
    const double c = std::cos(dead_yaw_), s = std::sin(dead_yaw_);
    wx = dead_x_ + c * sx - s * sy;
    wy = dead_y_ + s * sx + c * sy;
  }

  void worldToSensor(double wx, double wy, double & sx, double & sy) const
  {
    const double c = std::cos(-dead_yaw_), s = std::sin(-dead_yaw_);
    const double dx = wx - dead_x_, dy = wy - dead_y_;
    sx = c * dx - s * dy;
    sy = s * dx + c * dy;
  }

  void applyKalmanConfig()
  {
    kalman_.setProcessNoise(cfg_.kalman_q);
    kalman_.setMeasurementNoise(cfg_.kalman_r);
    kalman_.setGate(cfg_.kalman_gate);
    kalman_.setMeasurementNoiseScale(cfg_.kalman_r_ref_points);
  }

  void autoSelectFront(const ScanFrame & scan)
  {
    const double fov = cfg_.auto_front_fov_deg * M_PI / 180.0;
    double best_r = std::numeric_limits<double>::infinity();
    int best_i = -1;
    for (int i = 0; i < static_cast<int>(scan.ranges.size()); ++i) {
      const double r = scan.ranges[static_cast<size_t>(i)];
      if (!std::isfinite(r) || r > cfg_.auto_select_max_range) {
        continue;
      }
      const double a = scan.angleAt(i);
      if (std::abs(a) > fov) {
        continue;
      }
      if (r < best_r) {
        best_r = r;
        best_i = i;
      }
    }
    if (best_i < 0) {
      return;
    }
    const double a = scan.angleAt(best_i);
    target_x_ = best_r * std::cos(a);
    target_y_ = best_r * std::sin(a);
    target_valid_ = true;
    target_manual_ = false;
    lost_frames_ = 0;
    {
      double sx = target_x_, sy = target_y_;
      if (cfg_.filter_in_world) {
        sensorToWorld(target_x_, target_y_, sx, sy);
      }
      kalman_.setState(sx, sy);
    }
  }

  FollowConfig cfg_;
  KalmanFilter2D kalman_;
  bool target_valid_ = false;
  bool target_manual_ = false;
  int lost_frames_ = 0;
  double target_x_ = 1.0;
  double target_y_ = 0.0;
  int linear_dir_ = 0;
  int angular_dir_ = 0;
  double linear_integral_ = 0.0;
  std::chrono::steady_clock::time_point last_obs_time_;
  std::chrono::steady_clock::time_point last_control_time_;
  // inertial-frame target filtering support
  double odom_vx_ = 0.0, odom_vy_ = 0.0, odom_wz_ = 0.0;
  double dead_x_ = 0.0, dead_y_ = 0.0, dead_yaw_ = 0.0;
  bool has_odom_pose_ = false;
};

}  // namespace rs_follow

#endif  // RS_FOLLOW_FOLLOW_CONTROLLER_HPP
