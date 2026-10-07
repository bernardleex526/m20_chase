/**
 * @file speed_governor.hpp
 * @brief Time-to-collision speed limiting, replacing the fixed-radius e-stop.
 *
 * The original controller tripped a full emergency stop whenever the nearest
 * return outside the self-occlusion box came within `apf_emergency` (0.35 m).
 * That threshold is independent of speed, and the robot's own braking distance
 * at max speed is already larger than it:
 *
 *     d_brake = v^2 / (2 a_max) = 0.90^2 / 1.6 = 0.506 m  >  0.35 m
 *
 * so the alarm fired *inside* the physical stopping distance. It was also
 * omnidirectional: a wall 0.30 m to the SIDE produced a full stop even though
 * nothing blocked the path, and the rear was excluded entirely.
 *
 * This governor converts clearance into a speed limit instead, so the robot
 * slows down early and only hard-stops when a collision is unavoidable:
 *
 *     v * t_lat + v^2 / (2 a_max) <= d_clear - d_margin
 *     v_max = a_max * ( -t_lat + sqrt( t_lat^2 + 2 (d_clear - d_margin) / a_max ) )
 *
 * `d_clear` is the clearance in the direction the robot is about to move; the
 * caller selects which sector to measure it over (front when driving forward,
 * rear when reversing, all-round when turning in place).
 */

#ifndef RS_FOLLOW_SPEED_GOVERNOR_HPP
#define RS_FOLLOW_SPEED_GOVERNOR_HPP

#include <algorithm>
#include <cmath>
#include <limits>

namespace rs_follow
{

struct GovernorConfig
{
  bool enable = true;
  // Hard stop: below this clearance the robot commands zero, ignoring the
  // speed limit entirely. Safety floor, must stay well inside any brake margin.
  double d_hard = 0.25;
  // Clearance always kept between the robot outline and an obstacle.
  double d_margin = 0.15;
  // Total latency from "obstacle seen" to "deceleration applied": scan period,
  // control period, transport, actuator response. 0.2 s is a DESIGN ASSUMPTION
  // for the simulator (10 Hz scan + 50 Hz control, ideal actuator); a real
  // platform must measure this and raise it.
  double t_lat = 0.20;
  // Clearance kept when limiting speed, on top of the clearance at which the
  // robot would come to rest. The TTC curve is
  //     v * t_lat + v^2 / (2 a_max) <= d_clear - d_margin_slow
  // so d_margin_slow is the standoff the robot stops at, not a safety bubble
  // around obstacles it passes. Setting it large made the robot crawl at
  // 0.27 m/s through a 0.90 m doorway it fits through with 0.27 m to spare on
  // each side (the jamb is a STATIC obstacle that will never move into the
  // path, so treating it like a crossing pedestrian is over-conservative).
  // The margin therefore only has to cover the body's own positioning error and
  // the stop-distance model's optimism.
  double d_margin_slow = 0.05;
  // Deceleration used for planning (m/s^2). Keep <= the smoother's
  // max_linear_accel so the planned profile is actually achievable.
  double a_max = 0.80;
  // Speed at/above which the limit is considered non-binding.
  double v_cap = 1.50;
  // Omnidirectional safety floor (m of true surface clearance).
  //
  // The direction-gated clearance above answers "can I continue along this
  // heading". This answers a different and non-negotiable question: "is my body
  // already almost touching something, in ANY direction". Without it the robot
  // could graze: a crossing pedestrian entered the swept corridor between two
  // scans and was contacted at -0.07 m, because at the moment before contact it
  // was still just outside the corridor being measured.
  //
  // This is NOT the old fixed-radius ring that froze the robot at 0.35 m in
  // every direction (a wall 0.30 m to the side stopped a forward drive). That
  // was a large threshold applied regardless of geometry. This floor is small
  // enough that a 0.30 m side wall is comfortably clear of it, and it only
  // fires when the body genuinely has almost no room left.
  double emergency_floor = 0.10;
  // While waiting for a crossing obstacle, retreat when it is closer than this
  // (m) and at this speed (m/s), so it does not pass through where the robot
  // stands. Zero disables the retreat.
  double yield_distance = 1.2;
  double yield_speed = 0.35;
  // Extra time (s) added after a crossing obstacle has left the path before the
  // robot is allowed to arrive. Without it the robot aims to arrive exactly as
  // the obstacle clears, which is zero clearance by construction.
  double crossing_margin = 1.0;
  // Corridor widening used for the SPEED LIMIT. Must stay well below the
  // distance to a typical parallel wall (0.60 m here) or corridor walls count
  // as in-path and the robot brakes to a crawl everywhere; must be wide enough
  // that a crossing obstacle is seen before it enters the body's own corridor.
  double lateral_margin = 0.25;
  // Corridor widening used for the HARD STOP: kept tight, so the robot is not
  // stopped by geometry it would actually pass clear of (a door jamb).
  double stop_margin = 0.05;
  // Clearance beyond d_hard that must be regained before a hard stop releases.
  // Without this the clearance chatters across d_hard, re-arming the escape
  // latch every other cycle so the robot rocks instead of committing to a turn.
  double release_hysteresis = 0.15;
};

class SpeedGovernor
{
public:
  void setConfig(const GovernorConfig & cfg) {cfg_ = cfg;}
  const GovernorConfig & config() const {return cfg_;}

  /** True when clearance is inside the hard-stop floor. */
  bool hardStop(double d_clear) const
  {
    return cfg_.enable && std::isfinite(d_clear) && d_clear < cfg_.d_hard;
  }

  /**
   * @brief Maximum speed allowed for the given clearance (m/s).
   * Returns `v_cap` when clearance is unknown (no returns) or limiting is off.
   */
  /**
   * @brief Maximum speed allowed when the obstacle is itself approaching.
   *
   * The plain curve assumes a static obstacle, which is wrong for anything that
   * moves toward the robot: the closing rate is then (own speed + obstacle
   * speed), and budgeting only for the robot's own speed under-estimates the
   * required room by the obstacle's contribution. A pedestrian crossing at
   * 0.40 m/s while the robot drives at 0.90 m/s closes at 1.30 m/s, so the
   * robot needs 1.32 m of room where the static model thinks 0.74 m suffices --
   * which is how a 0.60 m gap got consumed to 0.000 m.
   *
   * Solving the same law for the total closing rate w = v + c_obs gives
   *     w_allow = a (-t + sqrt(t^2 + 2(d - margin)/a))
   * and the robot's own budget is whatever remains after the obstacle's share:
   *     v_allow = max(0, w_allow - c_obs)
   *
   * @param c_obs obstacle's closing rate toward the robot (m/s, >= 0)
   */
  double maxSpeed(double d_clear, double c_obs) const
  {
    const double w = maxSpeed(d_clear);
    if (c_obs <= 0.0) {
      return w;
    }
    return std::max(0.0, w - c_obs);
  }

  double maxSpeed(double d_clear) const
  {
    if (!cfg_.enable || !std::isfinite(d_clear)) {
      return cfg_.v_cap;
    }
    if (d_clear < cfg_.d_hard) {
      return 0.0;
    }
    const double slack = d_clear - cfg_.d_margin_slow;
    if (slack <= 0.0) {
      return 0.0;
    }
    const double t = cfg_.t_lat;
    const double v = cfg_.a_max *
      (-t + std::sqrt(t * t + 2.0 * slack / cfg_.a_max));
    return std::clamp(v, 0.0, cfg_.v_cap);
  }

private:
  GovernorConfig cfg_;
};

}  // namespace rs_follow

#endif  // RS_FOLLOW_SPEED_GOVERNOR_HPP