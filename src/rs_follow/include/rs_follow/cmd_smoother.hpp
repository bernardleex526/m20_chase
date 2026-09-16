/**
 * @file cmd_smoother.hpp
 * @brief Output-side velocity smoothing: acceleration limit + optional low-pass.
 *
 * The follow controller computes a *desired* Twist from the geometry each scan.
 * This class turns that desired command into the actually published command by
 * limiting linear/angular acceleration (slew rate) and optionally applying a
 * first-order low-pass filter. It runs on the control timer, so dt is exact.
 *
 * Emergency stops bypass the limiter and snap to zero immediately.
 */

#ifndef RS_FOLLOW_CMD_SMOOTHER_HPP
#define RS_FOLLOW_CMD_SMOOTHER_HPP

#include <algorithm>
#include <cmath>

#include <geometry_msgs/msg/twist.hpp>

namespace rs_follow
{

struct SmootherConfig
{
  double max_linear_accel = 0.8;    // m/s^2, applied to vx and vy
  double max_angular_accel = 1.5;   // rad/s^2, applied to wz
  double max_linear_jerk = 8.0;     // m/s^3, limits the change of acceleration
  double max_angular_jerk = 20.0;   // rad/s^3
  double cmd_filter_alpha = 0.0;    // 0 = off; (0,1] first-order low-pass
};

class CmdSmoother
{
public:
  explicit CmdSmoother(const SmootherConfig & cfg = SmootherConfig())
  : cfg_(cfg) {}

  void setConfig(const SmootherConfig & cfg) {cfg_ = cfg;}
  void reset()
  {
    out_ = geometry_msgs::msg::Twist();
    acc_x_ = acc_y_ = acc_w_ = 0.0;
  }
  const geometry_msgs::msg::Twist & output() const {return out_;}

  /**
   * @brief Advance the smoothed command toward `desired` by one control tick.
   * Acceleration is limited (slew) and its rate of change is jerk-limited, so
   * the published command is C1-continuous and does not step.
   */
  geometry_msgs::msg::Twist step(
    const geometry_msgs::msg::Twist & desired, double dt, bool emergency)
  {
    if (emergency) {
      out_ = geometry_msgs::msg::Twist();
      acc_x_ = acc_y_ = acc_w_ = 0.0;
      return out_;
    }
    if (dt <= 1e-4) {
      dt = 0.02;
    } else if (dt > 0.5) {
      dt = 0.5;
    }

    double tx = desired.linear.x;
    double ty = desired.linear.y;
    double tz = desired.angular.z;

    if (cfg_.cmd_filter_alpha > 0.0) {
      const double a = std::clamp(cfg_.cmd_filter_alpha, 0.0, 1.0);
      tx = out_.linear.x + a * (tx - out_.linear.x);
      ty = out_.linear.y + a * (ty - out_.linear.y);
      tz = out_.angular.z + a * (tz - out_.angular.z);
    }

    out_.linear.x = advance(out_.linear.x, tx, acc_x_, cfg_.max_linear_accel,
                            cfg_.max_linear_jerk, dt);
    out_.linear.y = advance(out_.linear.y, ty, acc_y_, cfg_.max_linear_accel,
                            cfg_.max_linear_jerk, dt);
    out_.angular.z = advance(out_.angular.z, tz, acc_w_, cfg_.max_angular_accel,
                             cfg_.max_angular_jerk, dt);
    return out_;
  }

private:
  // accelerate toward target with accel + jerk limits; updates acc in place
  static double advance(double cur, double target, double & acc, double a_max,
                        double j_max, double dt)
  {
    const double a_cmd = std::clamp((target - cur) / dt, -a_max, a_max);
    acc += std::clamp(a_cmd - acc, -j_max * dt, j_max * dt);
    return cur + acc * dt;
  }

  SmootherConfig cfg_;
  geometry_msgs::msg::Twist out_;
  double acc_x_ = 0.0, acc_y_ = 0.0, acc_w_ = 0.0;
};

}  // namespace rs_follow

#endif  // RS_FOLLOW_CMD_SMOOTHER_HPP
