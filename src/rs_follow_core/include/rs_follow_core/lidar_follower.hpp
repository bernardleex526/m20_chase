/**
 * @file lidar_follower.hpp
 * @brief The one class a non-ROS upper computer needs: point cloud in, twist out.
 *
 * This is the facade the evaluation report asked for. Everything below it is the
 * existing, simulation-validated algorithm (dual-band projection, Kalman target
 * filter, VFH+ gap steering, TTC speed governor, recovery and search FSMs); this
 * header only removes the ROS 2 dependency from the boundary so the same code
 * can be:
 *
 *   * linked into a plain C++ program or an embedded Linux application,
 *   * driven by a unit test with no middleware,
 *   * bound to Python (pybind11) without installing ROS 2,
 *   * wrapped by the ROS 2 node, which is now a thin adapter.
 *
 * TIMING MODEL. Perception (projection, tracking, obstacle estimation and the
 * control law) runs at the SCAN rate, exactly as the ROS node did in its cloud
 * callback; the output path (slip compensation, acceleration limiting, publish)
 * runs at the CONTROL rate. `update(dt)` is therefore safe to call at 50 Hz
 * while clouds arrive at 10 Hz: the expensive work only happens when a new scan
 * is pending. This preserves the frame-counting semantics of the original
 * controller (`lost_frames_timeout` is in FRAMES, so running the tracker at
 * 50 Hz instead of 10 Hz would have shortened the loss timeout fivefold).
 *
 * Usage:
 *     rs_follow::LidarFollower follower;
 *     follower.setConfig(cfg);        // defaults match config/follow_params.yaml
 *     follower.setEnabled(true);
 *     ...
 *     follower.setPointCloud(cloud);  // PointCloud, sensor frame
 *     follower.setOdometry(odom);     // optional but recommended
 *     follower.setRobotTwist(vx, vy, wz);
 *     rs_follow::Twist cmd = follower.update(dt);
 */

#ifndef RS_FOLLOW_CORE_LIDAR_FOLLOWER_HPP
#define RS_FOLLOW_CORE_LIDAR_FOLLOWER_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "rs_follow_core/cmd_smoother.hpp"
#include "rs_follow_core/dynamic_obstacle.hpp"
#include "rs_follow_core/follow_controller.hpp"
#include "rs_follow_core/multi_target_tracker.hpp"
#include "rs_follow_core/perf_monitor.hpp"
#include "rs_follow_core/pointcloud_scan.hpp"
#include "rs_follow_core/recovery_fsm.hpp"
#include "rs_follow_core/types.hpp"

namespace rs_follow
{

/** @brief Everything the follower needs, in one framework-independent struct. */
struct FollowerConfig
{
  ProjectionConfig projection;
  FollowConfig follow;
  SmootherConfig smoother;
  RecoveryConfig recovery;
  SearchConfig search;
  MultiTargetConfig multi_target;
  DynamicObstacleConfig dynamic_obstacle;

  // --- derived low band (see LidarFollower::deriveLowBand) ---
  bool auto_low_band = true;
  double sensor_height = 0.75;
  double ground_clearance = 0.10;
  double band_separation = 0.02;

  // --- safety / liveness ---
  bool active = false;          // safety default: disabled until explicitly enabled
  double control_rate_hz = 50.0;
  double cmd_timeout = 0.5;     // s without a scan before the command is zeroed
  double max_linear_cmd = 1.5;  // ceiling after slip compensation
  double max_angular_cmd = 2.0;

  // --- slip compensation ---
  bool compensate_slip = true;
  double slip_min_ratio = 0.3;
  double slip_max_ratio = 2.5;
  double slip_filter_alpha = 0.1;
  double slip_min_cmd = 0.05;
};

/** @brief Full state of one control step, for logging and for the ROS topics. */
struct FollowerStatus
{
  Twist cmd;                    // the command to send to the base
  FollowResult follow;          // tracker + planner diagnostics
  bool enabled = false;
  bool scan_fresh = false;      // a scan arrived within cmd_timeout
  bool emergency = false;
  bool recovery_active = false;
  RecoverState recovery_state = RecoverState::FOLLOW;
  int recovery_attempts = 0;
  SearchState search_state = SearchState::TRACK;
  int target_track_id = -1;     // multi-target id, -1 when single-target mode
  int track_count = 0;
  int obstacle_track_count = 0;
  double crossing_clearance = std::numeric_limits<double>::infinity();
  double crossing_closing = 0.0;
  double slip_lin_ratio = 1.0;
  double slip_ang_ratio = 1.0;
  double scan_age_s = 0.0;

  /** @brief Human-readable status string, same vocabulary as the ROS node. */
  std::string statusText() const
  {
    if (!enabled) {
      return "DISABLED";
    }
    if (emergency) {
      return "EMERGENCY_STOP";
    }
    if (follow.target_valid) {
      return follow.target_manual ? "TRACKING_MANUAL" : "TRACKING_AUTO";
    }
    return "NO_TARGET";
  }
};

/**
 * @brief Point-cloud person follower, ROS-free.
 */
class LidarFollower
{
public:
  LidarFollower() = default;
  explicit LidarFollower(const FollowerConfig & cfg) {setConfig(cfg);}

  void setConfig(const FollowerConfig & cfg)
  {
    cfg_ = cfg;
    deriveLowBand();
    controller_.setConfig(cfg_.follow);
    smoother_.setConfig(cfg_.smoother);
    recovery_.setConfig(cfg_.recovery);
    search_.setConfig(cfg_.search);
    // The multi-target tracker must exclude the same self-occlusion box as the
    // controller, or it clusters the robot's own chassis as a person.
    MultiTargetConfig mt = cfg_.multi_target;
    mt.frame_front = cfg_.follow.frame_front;
    mt.frame_back = cfg_.follow.frame_back;
    mt.frame_left = cfg_.follow.frame_left;
    mt.frame_right = cfg_.follow.frame_right;
    multi_.setConfig(mt);
    DynamicObstacleConfig doc = cfg_.dynamic_obstacle;
    doc.frame_front = cfg_.follow.frame_front;
    doc.frame_back = cfg_.follow.frame_back;
    doc.frame_left = cfg_.follow.frame_left;
    doc.frame_right = cfg_.follow.frame_right;
    doc.robot_length = cfg_.follow.robot_length;
    doc.robot_width = cfg_.follow.robot_width;
    dyn_.setConfig(doc);
  }

  const FollowerConfig & config() const {return cfg_;}
  FollowController & controller() {return controller_;}
  const FollowController & controller() const {return controller_;}
  MultiTargetTracker & multiTarget() {return multi_;}
  DynamicObstacleTracker & dynamicObstacles() {return dyn_;}
  PerfMonitor & perf() {return perf_;}

  // ---------------------------------------------------------------- inputs

  /**
   * @brief Store the latest cloud.
   * @return false if the cloud is malformed (the previous scan is kept)
   */
  bool setPointCloud(const PointCloud & cloud)
  {
    PerfScope scope(perf_, Stage::kProjection);
    if (!projectPointCloud(cloud, cfg_.projection, scan_)) {
      return false;
    }
    scan_valid_ = true;
    scan_age_ = 0.0;
    scan_pending_ = true;
    return true;
  }

  /** @brief Robot pose/velocity. Only the fields marked valid are consumed. */
  void setOdometry(const Odometry & odom)
  {
    odom_ = odom;
    if (odom.pose_valid) {
      controller_.setOdomPose(odom.x, odom.y, odom.yaw);
      pose_ = Pose2D{odom.x, odom.y, odom.yaw};
      have_pose_ = true;
    }
    if (odom.twist_valid) {
      controller_.setOdomTwist(odom.vx, odom.vy, odom.wz);
      robot_vx_ = odom.vx;
      robot_vy_ = odom.vy;
      robot_wz_ = odom.wz;
    }
  }

  /**
   * @brief What the base actually did (for slip compensation and the obstacle
   *        velocity estimator). Call with the measured body twist.
   */
  void setRobotTwist(double vx, double vy, double wz)
  {
    robot_vx_ = vx;
    robot_vy_ = vy;
    robot_wz_ = wz;
    controller_.setOdomTwist(vx, vy, wz);
    updateSlip(vx, vy, wz);
  }

  void setEnabled(bool on)
  {
    enabled_ = on;
    if (on) {
      scan_pending_ = scan_valid_;
    }
  }
  bool enabled() const {return enabled_;}
  void setControlMode(int mode) {control_mode_ = mode;}   // 0 = direct, 1 = follow
  int controlMode() const {return control_mode_;}
  void setDirectCommand(const Twist & t) {direct_ = t;}

  /** @brief Bind a target by point (sensor frame). @return snapped to a return. */
  bool bindTarget(double x, double y)
  {
    if (!scan_valid_) {
      return false;
    }
    if (cfg_.multi_target.enable) {
      multi_.lockNearest(x, y, cfg_.follow.bind_radius);
    }
    return controller_.bindTarget(x, y, scan_);
  }

  void clearTarget()
  {
    controller_.clearTarget();
    multi_.clearLock();
  }

  bool targetValid() const {return controller_.targetValid();}
  double targetX() const {return controller_.targetX();}
  double targetY() const {return controller_.targetY();}
  const ScanFrame & lastScan() const {return scan_;}
  const FollowResult & lastResult() const {return last_result_;}

  /** @brief Reset every internal state machine (target, FSMs, slip estimate). */
  void reset()
  {
    controller_.reset();
    recovery_.reset();
    search_.reset();
    multi_.reset();
    dyn_.reset();
    smoother_.reset();
    fsm_time_ = 0.0;
    lin_ratio_ = 1.0;
    ang_ratio_ = 1.0;
    last_pub_ = Twist();
    last_result_ = FollowResult();
    status_ = FollowerStatus();
    status_.enabled = enabled_;
    scan_valid_ = false;
    scan_pending_ = false;
  }

  // --------------------------------------------------------------- control

  /**
   * @brief One control step. Call at `control_rate_hz`.
   * @param dt seconds since the previous call
   * @return the command to publish
   */
  Twist update(double dt)
  {
    PerfScope total(perf_, Stage::kTotal);
    dt = std::clamp(dt, 1e-4, 0.5);
    fsm_time_ += dt;
    scan_age_ += dt;

    // Perception results are refreshed only on scan frames, so they are kept
    // across control ticks: resetting the whole status every tick made the
    // published track counts read zero on 4 out of every 5 control frames,
    // which is useless to a monitor.
    status_.cmd = Twist();
    status_.enabled = enabled_;

    const bool fresh = scan_valid_ && scan_age_ < cfg_.cmd_timeout;
    status_.scan_fresh = fresh;
    status_.scan_age_s = scan_age_;

    // ================= perception + control law, at the SCAN rate ==========
    bool emergency = false;
    if (scan_pending_ && fresh && enabled_ && control_mode_ != 0) {
      scan_pending_ = false;
      const double scan_dt = std::max(scan_age_, 1e-3);

      if (cfg_.multi_target.enable) {
        PerfScope mt(perf_, Stage::kController);
        multi_.update(scan_, scan_dt);
        status_.track_count = static_cast<int>(multi_.tracks().size());
        adoptMultiTarget();
      }
      if (cfg_.dynamic_obstacle.enable) {
        dyn_.update(scan_, scan_dt, robot_vx_, robot_vy_, robot_wz_);
        status_.obstacle_track_count = static_cast<int>(dyn_.tracks().size());
      }

      last_result_ = controller_.update(scan_, scan_dt);

      // Dynamic obstacle override: an obstacle that will be in the swept path
      // when the robot arrives caps the closing speed NOW, while there is still
      // braking room. This is the case a range-rate test cannot see (a crossing
      // pedestrian keeps a nearly constant range while its bearing sweeps).
      if (cfg_.dynamic_obstacle.enable && !last_result_.emergency_stop) {
        const double excl_r = cfg_.follow.exclude_target_from_obstacles
          ? cfg_.follow.target_radius + cfg_.follow.target_exclude_slack : 0.0;
        const CrossingResult cr = dyn_.crossingCheck(
          last_result_.cmd.linear_x, last_result_.cmd.linear_y,
          last_result_.target_x, last_result_.target_y, excl_r);
        status_.crossing_clearance = cr.clearance;
        status_.crossing_closing = cr.closing;
        const double v = last_result_.cmd.linear();
        if (cr.crossing && v > 1e-3 &&
          cr.clearance < cfg_.follow.governor.d_margin)
        {
          const double allow = governorSpeed(cr.clearance, cr.closing);
          const double s = std::clamp(allow / v, 0.0, 1.0);
          last_result_.cmd.linear_x *= s;
          last_result_.cmd.linear_y *= s;
          last_result_.detour = last_result_.detour;   // unchanged: heading kept
        }
      }

      if (last_result_.target_valid) {
        search_.noteSighting(fsm_time_, pose_.x, pose_.y);
      } else {
        search_.noteLost(fsm_time_);
      }
    }

    // ================= output path, at the CONTROL rate ====================
    Twist desired;
    if (enabled_ && fresh) {
      if (control_mode_ == 0) {
        desired = direct_;
      } else if (last_result_.target_valid) {
        if (last_result_.emergency_stop) {
          emergency = true;
          emergency_cmd_ = last_result_.cmd;
        } else {
          desired = last_result_.cmd;
        }
      } else {
        // ---- target search while the tracker has nothing ----
        double svx = 0.0, swz = 0.0;
        bool arrived = false;
        if (search_.update(fsm_time_, pose_.x, pose_.y, &svx, &swz, &arrived)) {
          desired.linear_x = svx;
          desired.angular_z = swz;
        } else {
          desired = Twist();      // COAST / HOLD: stand still
        }
      }

      // ---- stuck recovery ----
      //
      // This runs EVEN WHILE THE HARD STOP IS ACTIVE. Gating it on `!emergency`
      // was self-defeating: a wide obstacle (a 1 m step against a 0.36 m body)
      // leaves no traversable heading at the current pose, so the controller can
      // only rotate, the robot never translates, and the very state that needs
      // recovery was the one state recovery was not allowed to act in. The
      // machine's own outputs are bounded and it is the only path that reverses
      // out to where a gap becomes reachable again.
      double rvx = 0.0, rvy = 0.0, rwz = 0.0;
      const bool rcmd = (control_mode_ != 0 && last_result_.target_valid &&
        recovery_.update(fsm_time_, pose_.x, pose_.y, pose_.yaw,
                         desired.linear_x, desired.linear_y, desired.angular_z,
                         last_result_.clearance_rear, &rvx, &rvy, &rwz));
      if (rcmd) {
        desired.linear_x = rvx;
        desired.linear_y = rvy;
        desired.angular_z = rwz;
        emergency = false;          // recovery motion must reach the base
        recovery_active_ = true;
      } else {
        recovery_active_ = false;
      }
    } else if (enabled_ && !fresh) {
      desired = Twist();      // no sensor: stand still
    }

    // ---- slip / speed compensation (follow only) ----
    if (cfg_.compensate_slip && !emergency && control_mode_ != 0) {
      desired.linear_x /= lin_ratio_;
      desired.linear_y /= lin_ratio_;
      desired.angular_z /= ang_ratio_;
      desired.linear_x = std::clamp(desired.linear_x, -cfg_.max_linear_cmd, cfg_.max_linear_cmd);
      desired.linear_y = std::clamp(desired.linear_y, -cfg_.max_linear_cmd, cfg_.max_linear_cmd);
      desired.angular_z = std::clamp(desired.angular_z, -cfg_.max_angular_cmd, cfg_.max_angular_cmd);
    }

    // ---- output smoothing (emergency bypasses it: instant stop is the point) ----
    Twist out;
    {
      PerfScope sm(perf_, Stage::kSmoothing);
      out = smoother_.step(desired, dt, emergency);
    }
    if (emergency) {
      out = emergency_cmd_;
    }

    controller_.setOdomTwist(out.linear_x, out.linear_y, out.angular_z);
    last_pub_ = out;

    // ---- status ----
    status_.cmd = out;
    status_.follow = last_result_;
    status_.emergency = emergency;
    status_.recovery_active = recovery_active_;
    status_.recovery_state = recovery_.state();
    status_.recovery_attempts = recovery_.attempts();
    status_.search_state = search_.state();
    status_.slip_lin_ratio = lin_ratio_;
    status_.slip_ang_ratio = ang_ratio_;

    // Tick the perf monitor and STAGE any report due this step. The core has no
    // logger on purpose (it must work off-ROS), so the host collects the text
    // with perfReport() and decides where it goes.
    pending_perf_report_ = perf_.tick();
    return out;
  }

  /**
   * @brief The per-stage timing block due on the last update(), then cleared.
   *
   * Returns an empty string on every non-reporting step, so a host can call it
   * unconditionally each control tick:
   *     if (!report.empty()) { RCLCPP_INFO(get_logger(), "%s", report.c_str()); }
   */
  std::string perfReport()
  {
    std::string out;
    out.swap(pending_perf_report_);
    return out;
  }

  const FollowerStatus & status() const {return status_;}

private:
  /** @brief Speed the governor allows for a clearance and a closing rate. */
  double governorSpeed(double clearance, double closing) const
  {
    SpeedGovernor g;
    g.setConfig(cfg_.follow.governor);
    return g.maxSpeed(std::max(clearance, cfg_.follow.governor.d_hard), closing);
  }

  /**
   * @brief Derive the LOW obstacle band from the measured lidar height.
   *
   * The band must sit strictly ABOVE the floor: ground returns appear at sensor
   * z = -sensor_height, and a band that includes them fills every azimuth bin
   * with the floor. When that happens the person is masked out of the target
   * band (only the nearest return per bin survives) and the lock is lost.
   */
  void deriveLowBand()
  {
    if (!cfg_.projection.enable_low_band || !cfg_.auto_low_band) {
      return;
    }
    cfg_.projection.low_height_min = -cfg_.sensor_height + cfg_.ground_clearance;
    // Leave a DEAD ZONE between the bands. With only 0.05 m of separation the
    // top face of a low obstacle sits close enough to the target band that range
    // noise carries it over the boundary, where it is clustered as if it were
    // the person (observed as the tracker binding to a 0.30 m step).
    cfg_.projection.low_height_max = cfg_.projection.height_min - cfg_.band_separation;
    if (cfg_.projection.low_height_max <= cfg_.projection.low_height_min) {
      cfg_.projection.low_height_max = cfg_.projection.low_height_min + 0.05;
    }
  }

  /**
   * @brief Hand the multi-target selection to the single-target controller.
   *
   * The controller owns the control law and the safety layer; the multi-target
   * tracker owns IDENTITY. The division of labour matters:
   *
   *   * the tracker decides WHICH person is being followed, across frames;
   *   * the controller's own clustering + Kalman filter estimate that person's
   *     position from every scan, as it always did.
   *
   * So the tracker re-binds the controller only when the identity actually
   * changes -- on acquisition, and when the controller's estimate has drifted
   * off the locked track. It deliberately does NOT re-bind every frame:
   * `bindTarget` snaps to a raw scan return and re-seeds the Kalman state, so
   * doing it per frame would throw away the filter (and reset the integral term)
   * on every scan, leaving the robot following unfiltered measurement noise.
   */
  void adoptMultiTarget()
  {
    if (!cfg_.multi_target.enable) {
      return;
    }
    const double fov = cfg_.follow.auto_front_fov_deg * M_PI / 180.0;
    const int idx = multi_.select(fov, cfg_.follow.auto_select_max_range);
    if (idx < 0) {
      return;
    }
    const TargetTrack & t = multi_.tracks()[static_cast<size_t>(idx)];
    status_.target_track_id = t.id;

    if (!controller_.targetValid()) {
      // Acquisition: adopt the selected track and remember its identity.
      controller_.bindTarget(t.x, t.y, scan_);
      multi_.lockIndex(idx);
      return;
    }

    // Already tracking. Only intervene when the controller has drifted off the
    // locked track -- which is exactly the "followed the wrong person" failure
    // the multi-target layer exists to prevent.
    const TargetTrack * locked = multi_.lockedId() >= 0
      ? multi_.findById(multi_.lockedId()) : nullptr;
    if (!locked) {
      // The tracker lost its lock (all tracks aged out) but the controller still
      // has someone. Adopt whoever the tracker is now offering, and re-lock.
      controller_.bindTarget(t.x, t.y, scan_);
      multi_.lockIndex(idx);
      return;
    }
    if (locked->missed > 0) {
      return;   // coasting on prediction: leave the controller's estimate alone
    }
    const double drift = std::hypot(controller_.targetX() - locked->x,
                                    controller_.targetY() - locked->y);
    if (drift > cfg_.follow.target_radius) {
      controller_.bindTarget(locked->x, locked->y, scan_);
    }
  }

  /**
   * @brief Estimate the actual/commanded speed ratio (slip compensation).
   *
   * The robot is asked for a speed and reports a different one -- on a legged
   * base because of foot slip, on a wheeled one because of wheel slip. Scaling
   * the command by the inverse ratio makes the achieved speed match the request.
   */
  void updateSlip(double vx, double vy, double wz)
  {
    if (!cfg_.compensate_slip) {
      return;
    }
    const double cmdmag = std::hypot(last_pub_.linear_x, last_pub_.linear_y);
    const double actmag = std::hypot(vx, vy);
    if (cmdmag > cfg_.slip_min_cmd) {
      const double r = std::clamp(actmag / cmdmag, cfg_.slip_min_ratio, cfg_.slip_max_ratio);
      lin_ratio_ += cfg_.slip_filter_alpha * (r - lin_ratio_);
    }
    if (std::abs(last_pub_.angular_z) > cfg_.slip_min_cmd) {
      const double r = std::clamp(std::abs(wz) / std::abs(last_pub_.angular_z),
                                  cfg_.slip_min_ratio, cfg_.slip_max_ratio);
      ang_ratio_ += cfg_.slip_filter_alpha * (r - ang_ratio_);
    }
  }

  FollowerConfig cfg_;
  FollowController controller_;
  CmdSmoother smoother_;
  RecoveryMachine recovery_;
  TargetSearchMachine search_;
  MultiTargetTracker multi_;
  DynamicObstacleTracker dyn_;
  PerfMonitor perf_;

  ScanFrame scan_;
  bool scan_valid_ = false;
  bool scan_pending_ = false;
  double scan_age_ = 0.0;
  Odometry odom_;
  Pose2D pose_;
  bool have_pose_ = false;
  double robot_vx_ = 0.0, robot_vy_ = 0.0, robot_wz_ = 0.0;

  FollowResult last_result_;
  FollowerStatus status_;
  std::string pending_perf_report_;
  Twist last_pub_;
  Twist direct_;
  Twist emergency_cmd_;
  bool enabled_ = false;
  bool recovery_active_ = false;
  int control_mode_ = 1;      // 1 = FOLLOW, 0 = DIRECT
  double fsm_time_ = 0.0;
  double lin_ratio_ = 1.0;
  double ang_ratio_ = 1.0;
};

}  // namespace rs_follow

#endif  // RS_FOLLOW_CORE_LIDAR_FOLLOWER_HPP
