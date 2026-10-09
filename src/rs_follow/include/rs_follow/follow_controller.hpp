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
#include <cstdio>
#include <cstdint>
#include <cmath>
#include <limits>
#include <string>

#include <geometry_msgs/msg/twist.hpp>

#include "rs_follow/kalman_filter_2d.hpp"
#include "rs_follow/pointcloud_scan.hpp"
#include "rs_follow/speed_governor.hpp"
#include "rs_follow/vfh_planner.hpp"

namespace rs_follow
{

struct FollowConfig : BodyFrameConfig
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
  // Widest cluster (m) auto-select will accept as a person. A person is ~0.5 m
  // across; a wall reads far wider. Keeps the follower from locking onto a
  // surface when it has no target (measured: it locked the trap's back wall and
  // held station on it while the person stood 4.5 m away).
  double auto_max_target_width = 0.90;

  // manual binding
  double bind_radius = 0.6;

  // --- obstacle handling / safety ---
  // Exclude the followed person from the obstacle set. Without this the target
  // itself trips the safety bubble once the standoff drops below ~0.45 m.
  bool exclude_target_from_obstacles = true;
  // Radial slack (m) when deciding whether a return belongs to the target.
  double target_exclude_slack = 0.25;
  // Only the sector the robot is about to move into may limit it. The original
  // code used one omnidirectional scalar, so a wall 0.30 m to the side froze
  // the robot while the rear was ignored entirely.
  bool direction_gated_obstacles = true;
  // Half-width of the front/rear sectors (deg); 90 deg = a full half-plane.
  // Half-angle of the cone that may limit motion along the travel direction.
  // Must stay well under 90 deg: an obstacle at +-90 deg lies BESIDE the robot
  // and has no closing component along a forward drive, so it must only nudge
  // laterally (corridor following), not limit forward speed. At 90 deg a
  // corridor wall would freeze the robot -- exactly the S4/S5 failure.
  double sector_half_angle_deg = 60.0;
  // Time-to-collision speed governor (replaces the fixed-radius e-stop).
  GovernorConfig governor;

  // How long to keep rotating in place when the person is behind the robot
  // before allowing a slow reverse. Prevents an endless pirouette.
  double behind_rotate_timeout = 2.5;

  // Gap-based local steering. Without it the controller can only nudge or
  // stop; with it, a traversable gap is actually taken.
  VfhConfig vfh;
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
  // --- P0 diagnostics (published as debug topics / logged) ---
  double clearance_front = std::numeric_limits<double>::infinity();
  double clearance_rear = std::numeric_limits<double>::infinity();
  double clearance_used = std::numeric_limits<double>::infinity();
  double clearance_slow = std::numeric_limits<double>::infinity();
  double speed_limit = std::numeric_limits<double>::infinity();
  bool target_excluded = false;
  // --- P1 diagnostics ---
  double vfh_dir = 0.0;          // chosen heading (rad, sensor frame)
  double vfh_free = 0.0;         // clearance along it (m)
  bool vfh_traversable = false;
  bool vfh_blocked = false;
  double vfh_best_free = 0.0;
  double vfh_best_dir = 0.0;
  double detour = 0.0;           // |chosen - target| heading deviation (rad)
};

class FollowController
{
public:
  explicit FollowController(const FollowConfig & cfg = FollowConfig())
  : cfg_(cfg)
  {
    static_cast<BodyFrameConfig &>(cfg_) = normalizeBodyFrame(cfg_);
    applyKalmanConfig();
  }

  void setConfig(const FollowConfig & cfg)
  {
    cfg_ = cfg;
    static_cast<BodyFrameConfig &>(cfg_) = normalizeBodyFrame(cfg_);

    governor_.setConfig(cfg_.governor);
    // The planner's "traversable" clearance must be strictly larger than the
    // governor's hard-stop floor, otherwise it steers into the very band that
    // stops the robot: with d_safe 0.30 and d_hard 0.25 plus 0.15 hysteresis,
    // the chosen gap was inside the stop band and the robot ground forward until
    // true clearance fell to 0.24 m.
    if (cfg_.vfh.auto_d_safe) {
      cfg_.vfh.d_safe = cfg_.governor.d_hard +
        cfg_.governor.release_hysteresis + cfg_.vfh.d_safe_margin;
    }
    vfh_.setConfig(cfg_.vfh);
    applyKalmanConfig();
  }
  const FollowConfig & config() const {return cfg_;}

  /**
   * @brief Feed the robot's actual motion (odometry / published cmd_vel).
   * Used to keep the target filter in an inertial frame (see filter_in_world).
   */
    /**
   * @brief What limits motion along `dir`: braking room and true clearance.
   *
   * These are two DIFFERENT quantities, and conflating them is unsafe:
   *
   *   travel    = how far the body may advance along `dir` before its leading
   *               edge touches something. The speed governor compares THIS
   *               against its braking distance.
   *   true_clr  = surface-to-surface distance to the nearest return the swept
   *               body would actually reach. This is the physical clearance,
   *               and what a hard stop must be based on.
   *
   * For an obstacle dead ahead the two coincide. For one off to the side the
   * travel distance is larger (the body reaches it later). For one already
   * beside the robot the travel distance is ~0 while the true clearance is the
   * lateral gap. Using travel distance as the stop criterion made the robot
   * refuse to enter a 0.90 m doorway (the jamb is 0.27 m from the body --
   * comfortably clear -- but zero distance ahead along the travel axis). Using
   * true clearance as the speed criterion let it hit a crossing pedestrian
   * (0.40 m laterally is clear, but not at 0.9 m/s).
   *
   * The two corridors differ for the same reason: a hard stop only cares about
   * what the body will really hit, while the speed limit must already react to
   * something about to ENTER the path.
   */
  struct DirClearance
  {
    // How far the body may advance before its leading edge touches something
    // that is genuinely in its swept path (tight corridor). This is what the
    // HARD STOP uses: it must not fire on geometry the body would pass clear of.
    double travel = std::numeric_limits<double>::infinity();
    // The same, measured with a wider corridor so that something about to ENTER
    // the path is noticed early. This is what the SPEED limit uses.
    double travel_slow = std::numeric_limits<double>::infinity();
    // Surface-to-surface clearance to the nearest return the swept body reaches.
    double true_clr = std::numeric_limits<double>::infinity();
  };

  DirClearance directionalClearance(double dir) const
  {
    DirClearance out;
    if (obs_ranges_.empty()) {
      return out;
    }
    const double cd = std::cos(dir);
    const double sd = std::sin(dir);
    const int n = static_cast<int>(obs_ranges_.size());
    const double inc = (2.0 * M_PI) / n;
    const double support_travel = outlineSupport(dir);
    const double support_perp = outlineSupport(dir + 0.5 * M_PI);
    const double corridor_stop = support_perp + cfg_.governor.stop_margin;
    const double corridor_slow = support_perp + cfg_.governor.lateral_margin;
    for (int i = 0; i < n; ++i) {
      const double r = obs_ranges_[static_cast<size_t>(i)];
      if (!std::isfinite(r)) {
        continue;
      }
      const double a = -M_PI + inc * (i + 0.5);
      const double px = r * std::cos(a);
      const double py = r * std::sin(a);
      const double along = px * cd + py * sd;      // forward in travel frame
      const double lat = -px * sd + py * cd;       // sideways in travel frame
      const double alat = std::abs(lat);
      // A return only limits motion if the swept body would reach it. "Reach"
      // means it is not BEHIND the body: a return abreast of the robot (along
      // ~= 0, e.g. a corridor wall the robot is sliding along) still counts,
      // because the body is right next to it. Anything with along < -half_len
      // is genuinely behind and cannot be hit by moving forward.
      const double behind_limit = -0.5 * cfg_.robot_length;
      if (along < behind_limit) {
        continue;
      }
      if (alat <= corridor_stop) {
        // True clearance of the return itself, regardless of where along the
        // travel axis it sits: `r - support(a)` is the radial gap from the
        // outline to that surface. Off-axis returns are nearer than the
        // along-axis distance suggests, which is why a strafing robot could
        // graze a wall it had already drawn level with.
        const double clr = std::max(0.0, r - outlineSupport(a));
        if (clr < out.true_clr) {
          out.true_clr = clr;
        }
      }
      // Speed limit: anything in front whose lateral offset is within the
      // (modestly widened) swept corridor. `along > 0` is the whole gate --
      // deliberately NOT `along > support_travel`. An obstacle closer than the
      // body's half-length in front is the most dangerous case there is, and
      // gating it out left the speed limit at infinity while a crossing
      // pedestrian was 0.42 m away (observed as d_slow=-1.0, vlim=1.5, followed
      // by a real contact). The resulting travel distance is negative and clamps
      // to zero, i.e. "stop now", which is the correct answer.
      //
      // What keeps a parallel corridor wall out is the CORRIDOR WIDTH, not this
      // gate: the wall sits at alat ~ 0.60 m, outside corridor_slow (0.43 m).
      // Widening lateral_margin past that made corridor walls count as in-path
      // and froze the robot, which is why the margin stays modest.
      // Two corridors, two questions. Using the wide corridor for the hard stop
      // was wrong: a corridor wall 0.45 m to the side is outside the body's
      // swept path but inside the wide corridor, so travel computed to zero and
      // the robot stopped dead at the mouth of a 0.90 m doorway it fits through
      // comfortably.
      const double t = along - support_travel;
      if (alat <= corridor_stop && along > 0.0) {
        if (t < out.travel) {
          out.travel = t;
        }
      }
      if (alat <= corridor_slow && along > 0.0) {
        if (t < out.travel_slow) {
          out.travel_slow = t;
        }
      }
    }
    out.travel = std::max(0.0, out.travel);
    out.travel_slow = std::max(0.0, out.travel_slow);
    return out;
  }
  /**
   * @brief Exact distance from a point (robot frame) to the robot's rectangle.
   *
   * Zero when the point is inside the outline. This is the correct clearance
   * measure for a point obstacle.
   *
   * The obvious alternative -- the radial gap `r - support(a)`, where support(a)
   * is the box's extent along the point's bearing -- is NOT the distance. It
   * underestimates for points near a corner: a wall point at (0.20, 0.30) is
   * 0.12 m from the box edge, but the radial gap reports
   * hypot(0.20,0.30) - support(56 deg) = 0.361 - 0.322 = 0.039 m. That error
   * made the omnidirectional stop floor fire on geometry the robot was clear of,
   * which is precisely the false-stop bug this rewrite exists to remove.
   */
  double boxClearance(double px, double py) const
  {
    const double hx = 0.5 * cfg_.robot_length;
    const double hy = 0.5 * cfg_.robot_width;
    const double dx = std::max(0.0, std::abs(px) - hx);
    const double dy = std::max(0.0, std::abs(py) - hy);
    return std::hypot(dx, dy);
  }

  /** Support function of the robot outline along bearing `a` (m). */
  double outlineSupport(double a) const
  {
    return 0.5 * (cfg_.robot_length * std::abs(std::cos(a)) +
                  cfg_.robot_width * std::abs(std::sin(a)));
  }

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
    have_scan_stamp_ = false;
    require_rebind_ = true;
    behind_time_ = 0.0;
    prev_dir_ = 0.0;
    obs_ranges_.clear();
    prev_obs_ranges_.clear();
    obs_closing_.clear();
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
    behind_time_ = 0.0;
    prev_dir_ = 0.0;
    obs_ranges_.clear();
    prev_obs_ranges_.clear();
    obs_closing_.clear();
  }

  bool targetValid() const {return target_valid_;}
  bool targetManual() const {return target_manual_;}
  double targetX() const {return target_x_;}
  double targetY() const {return target_y_;}

  /**
   * @brief Bind a target from a clicked/selected point (lidar frame).
   * Returns true only after snapping to a target-band return within bind_radius.
   * Failure leaves the previous target and tracking state unchanged.
   */
  bool bindTarget(double x, double y, const ScanFrame & scan)
  {
    if (!std::isfinite(x) || !std::isfinite(y)) {
      return false;
    }
    double best_x = x, best_y = y;
    double best_d2 = cfg_.bind_radius * cfg_.bind_radius;
    bool snapped = false;

    for (int i = 0; i < static_cast<int>(scan.ranges.size()); ++i) {
      if (!std::isfinite(scan.ranges[static_cast<size_t>(i)]) ||
        scan.ranges[static_cast<size_t>(i)] <= 0.0f)
      {
        continue;
      }
      const double a = scan.angleAt(i);
      const double r = scan.ranges[static_cast<size_t>(i)];
      const double px = r * std::cos(a);
      const double py = r * std::sin(a);
      if (!std::isfinite(px) || !std::isfinite(py) ||
        (px > -cfg_.frame_back && px < cfg_.frame_front &&
        py > -cfg_.frame_right && py < cfg_.frame_left))
      {
        continue;
      }
      const double d2 = (px - x) * (px - x) + (py - y) * (py - y);
      if (d2 < best_d2) {
        best_d2 = d2;
        best_x = px;
        best_y = py;
        snapped = true;
      }
    }
    if (!snapped) {
      return false;
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
    require_rebind_ = false;
    return true;
  }

  /**
   * @brief Update tracking + control from the latest scan.
   */
  FollowResult update(const ScanFrame & scan)
  {
    FollowResult res;

    // Estimator and control time is sensor time, independent of playback speed.
    const int64_t stamp_ns = scan.stamp.nanoseconds();
    if (stamp_ns < 0 || (have_scan_stamp_ && stamp_ns <= last_scan_stamp_ns_))
    {
      linear_integral_ = 0.0;
      behind_time_ = 0.0;
      prev_obs_ranges_.clear();
      return res;
    }
    const double dt = have_scan_stamp_ ?
      static_cast<double>(stamp_ns - last_scan_stamp_ns_) * 1e-9 : 0.0;
    last_scan_stamp_ns_ = stamp_ns;
    have_scan_stamp_ = true;
    if (dt > 1.0) {
      clearTarget();
      require_rebind_ = true;
      return res;
    }

    if (cfg_.filter_in_world && !has_odom_pose_) {
      integrateOdom(dt);
    }

    // ---- target acquisition ----
    if (!target_valid_ && cfg_.auto_select_front && !require_rebind_) {
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
    // Nearest obstacle in true-clearance terms, with its bearing. Used to limit
    // the CLOSING speed toward it (see the governor section): a holonomic base
    // must respect obstacles in every direction, but only to the extent that it
    // is actually approaching them.


    const double tvx = target_x_;
    const double tvy = target_y_;
    const double tv_len = std::hypot(tvx, tvy);

    double left_y_min = -half_w;
    double right_y_min = half_w;

    // The followed person must NOT count as an obstacle, otherwise the target
    // itself trips the safety bubble once the standoff drops below ~0.45 m.
    // Exclusion is a radial window around the current target estimate, sized by
    // the cluster radius plus slack. It only removes returns in that window.
    const double tgt_excl_r = cfg_.target_radius + cfg_.target_exclude_slack;
    const double tgt_excl_r2 = tgt_excl_r * tgt_excl_r;
    int target_excluded_hits = 0;

    // Per-frame obstacle profile, one entry per azimuth bin. This MUST be
    // initialised here, once, before the scan loop. It was previously assigned
    // inside the loop, which reset the whole profile on every iteration and left
    // only the final bin populated -- so the local planner and the speed
    // governor both believed the world was empty (d_stop reported infinity with
    // an obstacle 0.36 m ahead) and the robot drove into things.
    obs_ranges_.assign(scan.ranges.size(),
                       std::numeric_limits<float>::infinity());
    // Per-bin APPROACH RATE (m/s, >= 0), from the change in range since the last
    // frame. A static world gives zero; a person walking at the robot gives
    // their walking speed. Needed because the TTC law otherwise budgets only for
    // the robot's own speed and under-estimates the room a closing obstacle
    // requires (see SpeedGovernor::maxSpeed(d, c_obs)).
    obs_closing_.assign(scan.ranges.size(), 0.0f);
    // Omnidirectional TRUE clearance: range minus the outline's extent along
    // that bearing, minimised over every obstacle return. `r - support(a)` is
    // the exact distance from the body rectangle to a point at (r, a), so this
    // is the real surface-to-surface gap in the nearest direction.
    double min_clearance_omni = std::numeric_limits<double>::infinity();
    double nearest_omni_bearing = 0.0;

    // Direction-gated clearance. `min_obstacle` (kept for logging/back-compat)
    // is the omnidirectional minimum; the values actually used for safety are
    // per-sector, so a wall beside the robot no longer freezes a forward drive
    // while the rear is still protected when reversing.
    // (sector_half_angle_deg is retained for the legacy APF path below)

    // The two bands are processed INDEPENDENTLY. Merging them (taking the
    // nearer range per bin) lets a floor return overwrite the person in that
    // bin, which silently emptied the target cluster and lost the lock. So:
    //   - target-band returns drive clustering, exclusion and the corridor;
    //   - the obstacle field is fed by whichever band is nearer, minus anything
    //     already attributed to the person.
    for (int i = 0; i < static_cast<int>(scan.ranges.size()); ++i) {
      const double a = scan.angleAt(i);
      const double ca = std::cos(a);
      const double sa = std::sin(a);

      const float rf_t = scan.rangeAt(i);
      const float rf_l = scan.lowRangeAt(i);
      const bool have_t = std::isfinite(rf_t);
      const bool have_l = std::isfinite(rf_l);
      if (!have_t && !have_l) {
        continue;
      }

      // ---------- target band: clustering / exclusion / corridor ----------
      // Returns inside the person's window are excluded from the corridor test
      // as well, so the person's own body does not look like a corridor wall.
      bool excluded_as_target = false;
      if (have_t) {
        const double px = rf_t * ca;
        const double py = rf_t * sa;
        const bool in_frame =
          (px > -cfg_.frame_back && px < cfg_.frame_front &&
          py > -cfg_.frame_right && py < cfg_.frame_left);
        const double d2t = (px - target_x_) * (px - target_x_) +
          (py - target_y_) * (py - target_y_);
        if (!in_frame && d2t < cfg_.target_radius * cfg_.target_radius) {
          centroid_x += px;
          centroid_y += py;
          ++in_target;
        } else if (!in_frame && cfg_.exclude_target_from_obstacles &&
          d2t < tgt_excl_r2)
        {
          // inside the person's window but outside the tight cluster radius
          excluded_as_target = true;
          ++target_excluded_hits;
        } else if (!in_frame && tv_len > 1e-6) {
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

      // ---------- obstacle field: nearer band, minus the person ----------
      const double d_obs = have_l ? static_cast<double>(rf_l)
        : static_cast<double>(rf_t);
      const bool obs_in_frame = [&]() {
        const double px = d_obs * ca;
        const double py = d_obs * sa;
        return (px > -cfg_.frame_back && px < cfg_.frame_front &&
                py > -cfg_.frame_right && py < cfg_.frame_left);
      }();

      // The person must be excluded from BOTH bands, not just the target band.
      // A standing person's legs fall inside the low band (world 0.10..0.30 m),
      // so the low-band return was being reported as an obstacle even though it
      // was the person: once the standoff dropped below ~0.5 m the robot
      // hard-stopped on its own follower. Measured as EMERGENCY_STOP with the
      // person at 0.48 m in the side-wall scenario.
      //
      // The test is a RAY test, not a distance-to-centre test: a return belongs
      // to the person if the ray passes within the person's radius of the
      // estimate AND the return sits at about the person's range along that
      // ray. That covers the torso, the legs and the far side of the body,
      // while a step or wall at a different range on the same bearing is still
      // seen.
      // Keep the per-bin obstacle range so the governor can ask for the clearance
    // along the ACTUAL travel direction. This robot is holonomic: it strafes,
    // and when it does, the limiting clearance is lateral, not the one along the
    // body x-axis. Using the x-axis value let a strafing robot graze obstacles
    // (measured 0.014 m and -0.015 m in the corridor and crossing scenarios).
    const double r_tgt_along = target_x_ * ca + target_y_ * sa;
      const double d_perp = std::abs(-target_x_ * sa + target_y_ * ca);
      const bool obs_is_person = cfg_.exclude_target_from_obstacles &&
        !obs_in_frame && (r_tgt_along > 0.0) &&
        (d_perp < tgt_excl_r) &&
        (std::abs(d_obs - r_tgt_along) < tgt_excl_r);
      (void)excluded_as_target;

      if (!obs_in_frame && !obs_is_person) {
        if (d_obs < min_obstacle) {
          min_obstacle = d_obs;
        }
        if (i < static_cast<int>(obs_ranges_.size())) {
          const size_t bi = static_cast<size_t>(i);
          // range rate: positive when the obstacle is getting closer
          const float prev = prev_obs_ranges_.empty()
            ? std::numeric_limits<float>::infinity()
            : prev_obs_ranges_[bi];
          if (std::isfinite(prev) && dt > 1e-3) {
            const double rate = (static_cast<double>(prev) - d_obs) / dt;
            // Clamp: scan-to-scan noise is ~0.1 m/s, and a genuine obstacle
            // cannot approach faster than a few m/s.
            obs_closing_[bi] = static_cast<float>(std::clamp(rate, 0.0, 5.0));
          }
          obs_ranges_[bi] = static_cast<float>(d_obs);
        }
        const double omni_clr = boxClearance(d_obs * ca, d_obs * sa);
        if (omni_clr < min_clearance_omni) {
          min_clearance_omni = omni_clr;
          nearest_omni_bearing = a;
        }

        // Travel-direction gating by SWEPT CORRIDOR, not by bearing cone.
        //
        // A bearing cone is the wrong test for a rectangular body. A wall
        // 0.60 m to the side sits at ~60 deg bearing, so a 60 deg half-angle
        // sector counts it as blocking even though the body's 0.18 m half-width
        // never reaches it -- that is what produced 4-5 s EMERGENCY_STOP
        // stretches in the corridor and pillar scenarios while the robot was
        // 0.6 m clear of everything.
        //
        // The correct question is: if the robot advances along +x, does the
        // body's swept rectangle (half-width W/2) hit this return? The lateral
        // offset of a return at range r and bearing a is r*sin(a) = py, so the
        // test is |py| <= half-width. Anything outside that is passed
        // alongside and must only nudge laterally.
        //
        // For the rear the same logic applies to a reverse along -x.
        const double clr = std::max(0.0, d_obs - outlineSupport(a));
        const double py_obs = d_obs * sa;
        const double px_obs = d_obs * ca;
        const double half_body_w = 0.5 * cfg_.robot_width;
        // Corridor test, with a lateral safety margin. The pure `|py| <= W/2`
        // form ignores anything that is about to ENTER the corridor: a
        // pedestrian walking across the path is momentarily just outside it, so
        // the gate opened, the robot accelerated, and the pedestrian arrived
        // (measured as a real contact at -0.002 m clearance). Widening the
        // corridor by the governor's margin makes the robot treat a crossing
        // obstacle as blocking before it gets there.
        const double corridor = half_body_w + cfg_.governor.d_margin;
        if (px_obs > 0.0 && std::abs(py_obs) <= corridor) {
          if (clr < res.clearance_front) {
            res.clearance_front = clr;
          }
        }
        if (px_obs < 0.0 && std::abs(py_obs) <= corridor) {
          if (clr < res.clearance_rear) {
            res.clearance_rear = clr;
          }
        }
        // Lateral returns feed the APF (so the robot can slide past) but are not
        // allowed to limit forward motion; only the travel-direction sector is.
        if (d_obs < cfg_.apf_influence && ca * d_obs > -0.1) {
          const double force = cfg_.apf_gain *
            (1.0 / d_obs - 1.0 / cfg_.apf_influence) / (d_obs * d_obs);
          repulse_x -= force * ca;
          repulse_y -= force * sa;
        }
      }
    }

    res.target_excluded = (target_excluded_hits > 0);

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
          kalman_.update(mx, my, dt, fx, fy, &accepted, in_target);
          worldToSensor(fx, fy, target_x_, target_y_);
        } else {
          kalman_.update(raw_x, raw_y, dt, fx, fy, &accepted, in_target);
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
    } else {
      ++lost_frames_;
      if (lost_frames_ > cfg_.lost_frames_timeout) {
        clearTarget();
        res.target_valid = false;
        return res;
      }
      if (cfg_.enable_kalman && in_target == 0) {
        double fx, fy;
        // Gated corrections already predict; only an absent measurement predicts here.
        kalman_.predictOnly(dt, fx, fy);
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
    // A target behind the robot is normally handled by rotating in place first.
    // That has no exit when the person stands still behind: the robot spins
    // forever at a 1 m standoff and never closes (observed as bearing -152 deg
    // with cmd=(0,0,-1.00) repeating, and as the side-wall and crowd scenarios
    // timing out). After `behind_rotate_timeout` of rotating, allow a slow
    // reverse so the robot backs toward the person instead of pirouetting.
    if (cfg_.rotate_in_place_behind && target_behind) {
      behind_time_ += dt;
    } else {
      behind_time_ = 0.0;
    }
    const bool behind_giveup =
      (cfg_.rotate_in_place_behind && target_behind &&
      behind_time_ > cfg_.behind_rotate_timeout);

    if (linear_dir_ == 0 || (cfg_.rotate_in_place_behind && target_behind && !behind_giveup)) {
      cmd.linear.x = 0.0;
    } else if (behind_giveup) {
      // back toward the person, gently, while still turning to face them
      cmd.linear.x = -std::min(cfg_.max_linear * 0.4, std::abs(dist_error) * cfg_.k_linear);
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

    // ---- gap-based local steering (VFH+) ----
    //
    // The stock law drives straight at the target bearing and relies on a weak
    // potential field, which cannot go around anything: an obstacle on the line
    // either nudges it slightly or stops it. Here the heading is chosen from
    // the scan itself -- the cheapest traversable direction that is wide enough
    // for the body -- so a gap in front of the robot is actually taken.
    double steer_ref = bearing;
    if (cfg_.vfh.enable) {
      VfhResult vfh_res = vfh_.plan(
        scan, bearing, prev_dir_,
        [this](double a) {return outlineSupport(a);});
      res.vfh_dir = vfh_res.direction;
      res.vfh_free = vfh_res.free;
      res.vfh_traversable = vfh_res.traversable;
      res.vfh_blocked = vfh_res.blocked;
      res.vfh_best_free = vfh_res.best_free;
      res.vfh_best_dir = vfh_res.best_dir;
      res.detour = std::abs(wrapPi(vfh_res.direction - bearing));
      if (vfh_res.traversable) {
        steer_ref = vfh_res.direction;
      } else {
        // Nothing is safely traversable. Aim at the widest direction so the
        // governor can creep toward it, and let the recovery FSM (P2) take over
        // if this persists. Never fall back to driving straight at the target,
        // which is what caused the "shove into the obstacle" behaviour.
        steer_ref = vfh_res.best_dir;
      }
      prev_dir_ = steer_ref;
    }

    // rotation toward the steering reference, with hysteresis
    const double steer_err = wrapPi(steer_ref);
    const double ab = std::abs(steer_err);
    if (angular_dir_ == 0) {
      if (ab >= cfg_.angular_deadband + cfg_.angular_hysteresis) {
        angular_dir_ = (steer_err > 0.0) ? 1 : -1;
      }
    } else if (ab <= cfg_.angular_deadband) {
      angular_dir_ = 0;
    }
    if (angular_dir_ == 0) {
      cmd.angular.z = 0.0;
    } else {
      cmd.angular.z = steer_err * cfg_.k_angular;
    }

    // A large detour means the robot is heading somewhere other than at the
    // person: do not drive forward hard while turned away, or the body sweeps
    // sideways into whatever the planner was avoiding.
    const bool detouring = res.detour > (60.0 * M_PI / 180.0);
    if (detouring) {
      cmd.linear.x = std::min(cmd.linear.x, 0.35 * cfg_.max_linear);
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

    // ================= collision avoidance: one constraint =================
    //
    // Earlier revisions layered three overlapping mechanisms here (a
    // direction-gated stop, a travel-distance speed limit, an omnidirectional
    // floor) and they fought each other: the stop fired on geometry the robot
    // would pass clear of, while the speed limit sat at infinity for a
    // pedestrian closing from the side. Both failure modes are the same
    // mistake -- reasoning about a scalar "clearance" instead of about the
    // robot's VELOCITY relative to each obstacle.
    //
    // There is exactly one constraint that matters, and it is per-obstacle:
    //
    //     the speed at which the robot APPROACHES an obstacle may not exceed
    //     what the clearance to that obstacle allows.
    //
    //     closing_i = max(0, v . u_i)            u_i = unit vector to obstacle i
    //     v_allow_i = governor.maxSpeed(clear_i)
    //     scale     = min over i of  v_allow_i / closing_i
    //
    // Scaling the whole velocity vector preserves the commanded heading, so the
    // robot slows down along the path it already chose. Obstacles it is receding
    // from contribute nothing (closing = 0), which is what lets it drive past a
    // wall 0.30 m to the side at full speed -- the original bug where a
    // fixed-radius ring froze the robot for geometry it was never going to hit.
    //
    // The hard stop is separate and deliberately narrow: it fires only when the
    // swept body truly cannot advance, so a 0.90 m doorway stays passable.
    const double vmag_cmd = std::hypot(cmd.linear.x, cmd.linear.y);

    // Obstacle bearings are needed per-obstacle, not just the nearest one, so
    // collect the (clearance, bearing) pairs that matter. The scan is 1440 bins
    // at 0.25 deg, so nearby bins are redundant; sampling every 8th bin (2 deg)
    // is ample and keeps this loop cheap at 50 Hz.
    // One obstacle bin, with everything the speed law needs: where it is, how
    // big the gap is, and how it is MOVING. The velocity is essential -- a
    // pedestrian crossing perpendicular to the robot's path has a range rate of
    // almost zero, so a range-rate-only test never notices it entering the path
    // (that was the last scenario failure: a real 1.8 cm graze).
    struct ObstacleSample
    {
      double clear;     // clearance from the body (m)
      double bearing;   // sensor-frame bearing (rad)
      double closing;   // range rate toward the robot (m/s, >= 0)
      double px, py;    // position in the sensor frame (m)
      double vx, vy;    // velocity in the sensor frame (m/s)
    };
    static thread_local std::vector<ObstacleSample> obs;
    obs.clear();
    const int n_bins = static_cast<int>(obs_ranges_.size());
    const int stride = 8;
    for (int i = 0; i < n_bins; i += stride) {
      const double r = obs_ranges_[static_cast<size_t>(i)];
      if (!std::isfinite(r)) {
        continue;
      }
      const double a = -M_PI + (2.0 * M_PI) * (i + 0.5) / n_bins;
      const double px = r * std::cos(a);
      const double py = r * std::sin(a);
      // Velocity from the per-bin range change. The bin index IS the bearing, so
      // the previous position is the same bearing at the previous range.
      double vx = 0.0, vy = 0.0;
      const float prev = prev_obs_ranges_.empty()
        ? std::numeric_limits<float>::infinity()
        : prev_obs_ranges_[static_cast<size_t>(i)];
      if (std::isfinite(prev) && dt > 1e-3) {
        const double pr = static_cast<double>(prev);
        vx = (px - pr * std::cos(a)) / dt;
        vy = (py - pr * std::sin(a)) / dt;
      }
      obs.push_back({std::max(0.0, r - outlineSupport(a)), a,
                     static_cast<double>(obs_closing_[static_cast<size_t>(i)]),
                     px, py, vx, vy});
    }

    // ---- hard stop: can the swept body advance at all? ----
    double d_advance = std::numeric_limits<double>::infinity();
    double d_advance_slow = std::numeric_limits<double>::infinity();
    double d_nearest = std::numeric_limits<double>::infinity();
    if (vmag_cmd > 0.02) {
      const double dir = std::atan2(cmd.linear.y, cmd.linear.x);
      const DirClearance dc = directionalClearance(dir);
      d_advance = dc.travel;            // tight corridor -> hard stop
      d_advance_slow = dc.travel_slow;  // wide corridor  -> speed limit
    }
    for (const auto & o : obs) {
      d_nearest = std::min(d_nearest, o.clear);
    }
    const bool turning_only = (vmag_cmd <= 0.02);
    double d_stop = turning_only ? d_nearest : d_advance;
    double scale = 1.0;


    // Omnidirectional hard-stop floor.
    //
    // The direction-gated stop answers "can I keep going this way", which is the
    // right question for speed but not sufficient for safety: the scan is a
    // 10 Hz snapshot, so at 0.9 m/s the robot advances 0.09 m between frames and
    // something entering its path can reach the body before the next frame
    // re-evaluates the corridor. Two measured grazes came from exactly that (a
    // crossing pedestrian contacted at -0.03 m; the trap's back wall contacted
    // 138 times).
    //
    // So there is also an absolute floor: if the body is within `emergency_floor`
    // of ANYTHING, in any direction, stop. The floor is deliberately SMALL
    // (0.08 m). The original code applied a large 0.35 m radius in every
    // direction, which froze the robot for a wall 0.30 m to the side it was never
    // going to hit; at 0.08 m a 0.30 m side wall still leaves 0.12 m of true
    // clearance and does not trigger it, while a real graze does.
    if (std::isfinite(min_clearance_omni) &&
      min_clearance_omni < cfg_.governor.emergency_floor)
    {
      d_stop = min_clearance_omni;
    }

    // Closing-speed limit against the NEAREST obstacle, independent of heading.
    //
    // The per-obstacle loop below only sees obstacles inside the travel
    // corridor, so a pedestrian walking INTO the robot from outside that
    // corridor is not limited until it is already at the floor -- measured as
    // clearance reaching 0.000 m with the pedestrian still advancing at
    // 0.40 m/s. The nearest obstacle is the one that matters most, so it gets an
    // unconditional closing-speed cap too.
    if (std::isfinite(min_clearance_omni) && vmag_cmd > 1e-6) {
      const double closing =
        cmd.linear.x * std::cos(nearest_omni_bearing) +
        cmd.linear.y * std::sin(nearest_omni_bearing);
      if (closing > 1e-6) {
        // Use the approach rate of the bin that is actually nearest.
        double c_near = 0.0;
        for (const auto & o : obs) {
          if (std::abs(o.clear - min_clearance_omni) < 1e-6) {
            c_near = std::max(c_near, o.closing);
          }
        }
        const double allow = governor_.maxSpeed(
          std::max(min_clearance_omni, cfg_.governor.d_hard), c_near);
        if (allow < closing) {
          scale = std::min(scale, allow / closing);
        }
      }
    }
    res.clearance_used = d_stop;
    prev_obs_ranges_ = obs_ranges_;   // for next frame's range-rate estimate

    // ---- speed limit: min over obstacles of allowed closing speed ----
    if (vmag_cmd > 1e-6) {
      for (const auto & o : obs) {
        const double closing =
          cmd.linear.x * std::cos(o.bearing) + cmd.linear.y * std::sin(o.bearing);
        if (closing <= 1e-6) {
          continue;                     // receding from this one: ignore it
        }
        // Evaluate the curve at no less than d_hard. maxSpeed() is exactly zero
        // below d_hard, which is right for the hard-stop test but wrong here:
        // applying it to the closing rate froze the robot outright while it was
        // merely passing within 0.23 m of something.
        const double allow = governor_.maxSpeed(
          std::max(o.clear, cfg_.governor.d_hard), o.closing);
        if (allow < closing) {
          scale = std::min(scale, allow / closing);
        }
      }
      // ---- "will it be in my way when I get there?" ----
      //
      // This is the piece a range-rate test cannot provide. A pedestrian walking
      // across the robot's path keeps a nearly constant RANGE while its BEARING
      // sweeps, so its range rate is ~0 and no closing-speed test reacts. But it
      // is about to occupy the path. The honest test is to predict where it will
      // be when the robot arrives, and if that predicted position is inside the
      // swept corridor, treat it as an obstacle at its current along-axis
      // distance -- i.e. the robot must be able to stop before reaching it.
      if (vmag_cmd > 1e-6) {
        const double dir = std::atan2(cmd.linear.y, cmd.linear.x);
        const double cd = std::cos(dir), sd = std::sin(dir);
        const double corridor =
          outlineSupport(dir + 0.5 * M_PI) + cfg_.governor.lateral_margin;
        for (const auto & o : obs) {
          const double along = o.px * cd + o.py * sd;
          if (along <= 0.0) {
            continue;                       // behind the travel direction
          }
          const double lat = -o.px * sd + o.py * cd;


          // Obstacle inside the swept corridor: ordinary braking-distance rule.
          if (std::abs(lat) < corridor) {
            const double room = std::max(0.0, along - outlineSupport(dir));
            const double allow = governor_.maxSpeed(
              std::max(room, cfg_.governor.d_hard));
            if (vmag_cmd > allow && vmag_cmd > 1e-6) {
              scale = std::min(scale, allow / vmag_cmd);
            }
          }
        }
      }
      // Also cap by the braking room along the chosen path (wide corridor), so
      // something about to step into the path is reacted to before it arrives.
      if (std::isfinite(d_advance_slow)) {
        // Clamp to d_hard for the same reason as the per-obstacle loop:
        // maxSpeed() is exactly zero below d_hard, and applying that here
        // produced scale = 0 -- a hard freeze -- whenever the wide corridor
        // clipped something merely near the body. Stopping is the hard stop's
        // decision, not this one.
        const double allow =
          governor_.maxSpeed(std::max(d_advance_slow, cfg_.governor.d_hard));
        if (vmag_cmd > allow && vmag_cmd > 1e-6) {
          scale = std::min(scale, allow / vmag_cmd);
        }
      }
    }
    res.clearance_slow = scale < 1.0 ? scale : std::numeric_limits<double>::infinity();

    if (governor_.hardStop(d_stop)) {
      res.emergency_stop = true;
      res.speed_limit = 0.0;
      res.cmd = geometry_msgs::msg::Twist{};
      return res;
    }

    // Continuous limit: slow down early instead of stopping late. Scaling the
    // (vx, vy) vector preserves the commanded heading while capping its speed.
    res.speed_limit = scale * vmag_cmd;
    if (scale < 1.0) {
      cmd.linear.x *= scale;
      cmd.linear.y *= scale;
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

  /**
   * @brief Pick a person to follow from the front field of view.
   *
   * Selects the nearest CLUSTER THAT COULD BE A PERSON, not the nearest return.
   *
   * Picking the nearest return is wrong and was measured doing real damage: in a
   * U-shaped trap the closest thing ahead is the trap's back wall, so the node
   * locked onto the wall, reported TRACKING_AUTO, and held a tidy 0.95 m
   * standoff from it while the actual person stood 4.5 m away. Every A1-A4
   * criterion passed, because holding station from the wrong object is still a
   * perfect standoff -- the acceptance suite simply never asked whether the
   * thing being followed was a person.
   *
   * Two cheap tests separate a person from a wall, and neither needs a learned
   * model:
   *
   *   * A cluster that reaches the edge of the field of view is a SURFACE
   *     continuing beyond it. A person standing free never spans the whole FOV.
   *   * A person is about 0.5 m wide. Convert the cluster's angular width to a
   *     physical width at its range and reject anything much wider -- this is
   *     range-independent, so it works both close up and far away.
   */
  void autoSelectFront(const ScanFrame & scan)
  {
    const double fov = cfg_.auto_front_fov_deg * M_PI / 180.0;
    const int n = static_cast<int>(scan.ranges.size());
    if (n <= 0) {
      return;
    }
    const double bin_ang = std::abs(scan.angle_increment);

    // Group contiguous in-range bins into clusters, then judge each cluster.
    double best_r = std::numeric_limits<double>::infinity();
    double best_angle = 0.0;
    bool found = false;
#ifdef RS_FOLLOW_DEBUG_AUTOSELECT
    int runs = 0, hits = 0;
    double last_w = 0.0, last_r = 0.0;
    int last_bins = 0;
#endif

    int i = 0;
    while (i < n) {
      const double r = scan.ranges[static_cast<size_t>(i)];
      const double a = scan.angleAt(i);
      // Self-occlusion must be applied HERE too, exactly as the tracking loop
      // does. Without it the selector sees the robot's own chassis: the lidar
      // sits above the body, whose top surface still falls inside the target
      // height band, and the chassis is nearer than any person, inside the field
      // of view, and only ~0.6 m wide in angular terms -- so it passes the width
      // test and gets selected. The tracking loop then rejects those same
      // returns as self-occlusion, the cluster comes out empty, the measurement
      // fails, and the lock is dropped again next frame. That is the mechanism
      // behind the observed "target=LOCK ... pts=0" during the search: the robot
      // kept selecting itself instead of the person 1.2 m away.
      const auto self_occluded = [&](int idx) {
          const double rr = scan.ranges[static_cast<size_t>(idx)];
          const double aa = scan.angleAt(idx);
          const double px = rr * std::cos(aa);
          const double py = rr * std::sin(aa);
          return (px > -cfg_.frame_back && px < cfg_.frame_front &&
                 py > -cfg_.frame_right && py < cfg_.frame_left);
        };
      if (!std::isfinite(r) || r > cfg_.auto_select_max_range ||
        std::abs(a) > fov)
      {
        ++i;
        continue;
      }
      // Grow the run, tolerating holes. Two separate things create holes and
      // both must be stepped over rather than treated as the end of a cluster:
      //
      //   * SELF-OCCLUSION. The chassis covers a contiguous arc in the middle of
      //     the view. Ending the run there splintered a person-sized cluster
      //     into single-bin fragments -- measured as 47 "clusters" of 1 bin and
      //     0.00-0.01 m width, so nothing was ever selected.
      //   * SPARSE SAMPLING. The projection has 1440 bins (0.25 deg) but the
      //     LiDAR emits 720 rays (0.5 deg), so every second bin is empty by
      //     construction. Requiring two ADJACENT populated bins can therefore
      //     never be satisfied, which is what made this fail even with the
      //     chassis correctly excluded.
      //
      // A gap is bridged only up to a few bins. A person's visible arc is ~27 deg
      // at 1.2 m (over a hundred bins), while two people standing half a metre
      // apart 3 m away are ~9 deg apart, so small-gap bridging merges sampling
      // holes without merging distinct people.
      const int max_gap = 4;
      int j = i;
      int last_populated = i;
      double min_r = r;
      int used = 1;
      while (j + 1 < n && (j - last_populated) <= max_gap) {
        const int k = j + 1;
        const double rk = scan.ranges[static_cast<size_t>(k)];
        const double ak = scan.angleAt(k);
        if (!std::isfinite(rk) || rk > cfg_.auto_select_max_range ||
          std::abs(ak) > fov)
        {
          if (j - last_populated >= max_gap) {
            break;
          }
          ++j;
          continue;
        }
        ++j;
        if (!self_occluded(k)) {
          min_r = std::min(min_r, rk);
          ++used;
          last_populated = j;
        }
      }
      const int width_bins_visible = used;

      const double a0 = scan.angleAt(i);
      const double a1 = scan.angleAt(last_populated);
      // The span must likewise cover only VISIBLE bins, or the chassis in the
      // middle of the cluster would inflate the apparent width.
      const double span = std::abs(a1 - a0) + bin_ang;
      // The discriminator is PHYSICAL WIDTH, and it is sufficient on its own.
      //
      // A cluster's angular span converted to metres at its range separates a
      // person from a surface regardless of distance: a wall spans the whole
      // field of view (2*2.2*sin(40 deg) = 2.8 m at 2.2 m, 10 m at 8 m) while a
      // person is ~0.5 m. Being range-independent is the point -- a distance
      // threshold would only work at one range.
      //
      // An additional "the cluster must not touch the FOV edge" heuristic was
      // tried and REMOVED: it rejected a person the moment they entered the
      // field of view from the side, which is exactly how a spinning robot first
      // sees them, and it silently broke reacquisition after a target loss.
      const double width_m = 2.0 * min_r * std::sin(std::min(span, M_PI) / 2.0);

#ifdef RS_FOLLOW_DEBUG_AUTOSELECT
      ++runs;
      last_w = width_m;
      last_bins = width_bins_visible;
      last_r = min_r;
#endif
      if (width_m <= cfg_.auto_max_target_width && width_bins_visible >= 2) {
#ifdef RS_FOLLOW_DEBUG_AUTOSELECT
        ++hits;
#endif
        if (min_r < best_r) {
          best_r = min_r;
          best_angle = 0.5 * (a0 + a1);
          found = true;
        }
      }
      i = j;
    }

    if (!found) {
#ifdef RS_FOLLOW_DEBUG_AUTOSELECT
      std::fprintf(stderr, "[autoselect] no candidate; runs=%d last(width=%.2f "
                   "bins=%d min_r=%.2f)\n", runs, last_w, last_bins, last_r);
#endif
      return;
    }
#ifdef RS_FOLLOW_DEBUG_AUTOSELECT
    std::fprintf(stderr, "[autoselect] picked r=%.2f ang=%.1fdeg hits=%d\n",
                 best_r, best_angle * 180.0 / M_PI, hits);
#endif
    const double a = best_angle;
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
  SpeedGovernor governor_;
  VfhPlanner vfh_;
  double prev_dir_ = 0.0;
  double behind_time_ = 0.0;
  std::vector<float> obs_ranges_;
  std::vector<float> prev_obs_ranges_;
  std::vector<float> obs_closing_;
  bool target_valid_ = false;
  bool target_manual_ = false;
  int lost_frames_ = 0;
  double target_x_ = 1.0;
  double target_y_ = 0.0;
  int linear_dir_ = 0;
  int angular_dir_ = 0;
  double linear_integral_ = 0.0;
  bool have_scan_stamp_ = false;
  bool require_rebind_ = false;
  int64_t last_scan_stamp_ns_ = 0;
  // inertial-frame target filtering support
  double odom_vx_ = 0.0, odom_vy_ = 0.0, odom_wz_ = 0.0;
  double dead_x_ = 0.0, dead_y_ = 0.0, dead_yaw_ = 0.0;
  bool has_odom_pose_ = false;
};

}  // namespace rs_follow

#endif  // RS_FOLLOW_FOLLOW_CONTROLLER_HPP
