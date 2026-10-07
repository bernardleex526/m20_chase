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

#include "rs_follow_core/types.hpp"

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
    out_ = Twist();
    acc_x_ = acc_y_ = acc_w_ = 0.0;
  }
  const Twist & output() const {return out_;}

  /**
   * @brief Advance the smoothed command toward `desired` by one control tick.
   * Acceleration is limited (slew) and its rate of change is jerk-limited, so
   * the published command is C1-continuous and does not step.
   */
  Twist step(
    const Twist & desired, double dt, bool emergency)
  {
    if (emergency) {
      out_ = Twist();
      acc_x_ = acc_y_ = acc_w_ = 0.0;
      return out_;
    }
    if (dt <= 1e-4) {
      dt = 0.02;
    } else if (dt > 0.5) {
      dt = 0.5;
    }

    double tx = desired.linear_x;
    double ty = desired.linear_y;
    double tz = desired.angular_z;

    if (cfg_.cmd_filter_alpha > 0.0) {
      const double a = std::clamp(cfg_.cmd_filter_alpha, 0.0, 1.0);
      tx = out_.linear_x + a * (tx - out_.linear_x);
      ty = out_.linear_y + a * (ty - out_.linear_y);
      tz = out_.angular_z + a * (tz - out_.angular_z);
    }

    out_.linear_x = advance(out_.linear_x, tx, acc_x_, cfg_.max_linear_accel,
                            cfg_.max_linear_jerk, dt);
    out_.linear_y = advance(out_.linear_y, ty, acc_y_, cfg_.max_linear_accel,
                            cfg_.max_linear_jerk, dt);
    out_.angular_z = advance(out_.angular_z, tz, acc_w_, cfg_.max_angular_accel,
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
  Twist out_;
  double acc_x_ = 0.0, acc_y_ = 0.0, acc_w_ = 0.0;
};

}  // namespace rs_follow

#endif  // RS_FOLLOW_CMD_SMOOTHER_HPP
