/**
 * @file dynamic_obstacle.hpp
 * @brief Track moving obstacles and predict where they will be, not just where
 *        they are.
 *
 * WHY THIS EXISTS
 * ---------------
 * The stock safety layer reasons about the CURRENT scan: clearance to the
 * nearest return, and a range rate per bin. Both fail on the case that matters
 * most for a follower -- a person walking ACROSS the robot's path:
 *
 *   * their RANGE barely changes, so a range-rate test reads ~0 and the robot
 *     does not slow down at all;
 *   * their BEARING sweeps, so the bin they occupy changes every frame and the
 *     per-bin range difference compares against an empty bin, which is noise;
 *   * by the time they are inside the swept corridor, the robot's braking
 *     distance exceeds the remaining gap. In simulation this was measured as a
 *     real contact at -0.002 m and a 1.8 cm graze.
 *
 * The existing controller has partial, ad-hoc patches for this (a "will it be in
 * my way when I get there?" check, plus a nearest-obstacle velocity estimate
 * from the position change of the single nearest return). Those work only for
 * the nearest return and only for a single frame's worth of motion.
 *
 * This module does it properly and generally:
 *   1. cluster the scan into obstacles,
 *   2. associate clusters with tracks across frames,
 *   3. estimate each track's velocity in the SENSOR frame, with the robot's own
 *      motion removed so a static wall reads zero,
 *   4. extrapolate each track forward by a lookahead horizon,
 *   5. answer the two questions the controller actually asks:
 *        - does this obstacle enter my swept path before I get there?
 *        - how fast are we closing, right now and at the closest approach?
 *
 * Velocity is estimated from the cluster CENTROID's position change, not from a
 * per-bin range difference. That is the fix for the crossing pedestrian: a
 * centroid moves smoothly through bearing while the bin index jumps around.
 */

#ifndef RS_FOLLOW_CORE_DYNAMIC_OBSTACLE_HPP
#define RS_FOLLOW_CORE_DYNAMIC_OBSTACLE_HPP

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "rs_follow_core/pointcloud_scan.hpp"

namespace rs_follow
{

/** @brief One tracked obstacle with an estimated velocity, sensor frame. */
struct ObstacleTrack
{
  int id = -1;
  double x = 0.0;         // last MEASURED position (m), sensor frame
  double y = 0.0;
  // The obstacle's OWN velocity in the world frame (m/s), i.e. with the robot's
  // motion removed. A static wall therefore reads (0, 0) no matter how fast the
  // robot drives at it, which is what makes `moving()` meaningful.
  double vx = 0.0;
  double vy = 0.0;
  double radius = 0.0;    // apparent half-width (m)
  int points = 0;
  int missed = 0;
  double age_s = 0.0;
  bool measured = false;

  double range() const {return std::hypot(x, y);}
  double bearing() const {return std::atan2(y, x);}
  /** @brief Speed in the world frame (m/s). */
  double speed() const {return std::hypot(vx, vy);}
  /** @brief True when the obstacle is moving enough to matter (m/s). */
  bool moving(double threshold = 0.15) const {return speed() > threshold;}

  // Predicted position for THIS frame, relative to the robot. Written by the
  // tracker during update(); `px_`/`py_` are internal, not an API.
  double px_ = 0.0;
  double py_ = 0.0;

  /** @brief Position after `t` s assuming the obstacle keeps its world velocity. */
  void predictWorld(double t, double & px, double & py) const
  {
    px = x + vx * t;
    py = y + vy * t;
  }
};

struct DynamicObstacleConfig
{
  bool enable = true;
  // --- clustering (obstacles use BOTH bands; the low band is often all a step
  //     or a small object has) ---
  double max_range = 6.0;      // obstacles beyond this do not constrain (m)
  int max_gap_bins = 4;        // bridge sampling / self-occlusion holes
  int min_points = 2;
  double min_width = 0.02;     // narrower than this is noise (m)
  // Wider than this is a long wall. A wall's centroid is a poor stand-in for its
  // geometry, and its velocity is trivially zero, so it is left to the static
  // safety layer instead of being tracked here. Kept generous (6 m) so that a
  // genuinely large MOVING obstacle -- a cart, a vehicle -- is still tracked.
  double max_width = 6.00;
  // --- association ---
  double assoc_gate = 0.60;    // m, against the PREDICTED position
  int max_missed = 10;
  int max_tracks = 24;
  // --- velocity estimation ---
  double vel_alpha = 0.4;      // low-pass on the estimated velocity
  double max_speed = 4.0;      // reject estimates above this (m/s)
  // --- prediction ---
  double lookahead = 1.5;      // s of extrapolation for the "will it be there"
  double corridor_margin = 0.25;  // extra half-width when testing the path
  // Robot outline, used to turn a range into a true clearance.
  double robot_length = 0.62;
  double robot_width = 0.36;
  // --- self-occlusion box ---
  double frame_front = 0.25;
  double frame_back = 0.45;
  double frame_left = 0.25;
  double frame_right = 0.25;
};

/** @brief Answer to "is anything crossing my intended path?" */
struct CrossingResult
{
  bool crossing = false;      // an obstacle enters the swept path in time
  double t_arrive = 0.0;      // s until the robot would reach it
  double clearance = std::numeric_limits<double>::infinity();  // m, at closest
  double closing = 0.0;       // m/s closing rate toward it
  int id = -1;                // track responsible
  double ox = 0.0, oy = 0.0;  // its position at closest approach
};

/**
 * @brief Maintains velocity estimates for the obstacles in the scan.
 */
class DynamicObstacleTracker
{
public:
  void setConfig(const DynamicObstacleConfig & cfg) {cfg_ = cfg;}
  const DynamicObstacleConfig & config() const {return cfg_;}
  const std::vector<ObstacleTrack> & tracks() const {return tracks_;}

  void reset()
  {
    tracks_.clear();
    next_id_ = 0;
    prev_vx_ = prev_vy_ = 0.0;
    have_prev_twist_ = false;
  }

  /**
   * @brief Update obstacle tracks from one scan.
   * @param scan        polar scan (both bands are used)
   * @param dt          seconds since the previous call
   * @param robot_vx    robot body velocity (m/s), sensor frame
   * @param robot_vy    robot body velocity (m/s)
   * @param robot_wz    robot yaw rate (rad/s)
   *
   * VELOCITY MODEL. `x/y` is the last MEASURED position in the sensor frame and
   * `vx/vy` is the obstacle's own velocity in the WORLD frame. Writing the world
   * velocity rather than the sensor-frame one matters: the sensor frame
   * translates with the robot, so a static wall would otherwise appear to rush
   * at the robot at the robot's own speed and every obstacle would look moving.
   *
   * With the robot's body velocity (vx_b, vy_b) and yaw rate wz, a point at
   * (x, y) in the sensor frame has the sensor frame moving under it at
   *     own_v = (vx_b - wz*y, vy_b + wz*x)
   * so the obstacle's world velocity is the observed sensor-frame motion plus
   * that term. A wall therefore cancels to zero, and a pedestrian walking at
   * 0.4 m/s reads 0.4 m/s regardless of how the robot is driving.
   */
  void update(const ScanFrame & scan, double dt,
              double robot_vx, double robot_vy, double robot_wz)
  {
    if (!cfg_.enable) {
      return;
    }
    dt = std::clamp(dt, 0.001, 0.5);

    std::vector<Cluster> clusters = extractClusters(scan);

    // --- predict existing tracks forward for association only ---
    //
    // The predicted position is where the obstacle will be RELATIVE TO THE
    // ROBOT, so the robot's own motion is part of the relative velocity. Using
    // the world velocity alone would leave a static obstacle's prediction
    // standing still in the sensor frame while it actually sweeps toward the
    // robot, and the association gate would then miss it every frame.
    for (auto & t : tracks_) {
      double own_vx = 0.0, own_vy = 0.0;
      sensorFrameMotion(t.x, t.y, robot_vx, robot_vy, robot_wz, own_vx, own_vy);
      const double rel_vx = t.vx - own_vx;
      const double rel_vy = t.vy - own_vy;
      t.px_ = t.x + rel_vx * dt;
      t.py_ = t.y + rel_vy * dt;
      t.age_s += dt;
      t.measured = false;
    }

    // --- associate ---
    std::vector<bool> used(tracks_.size(), false);
    for (const auto & c : clusters) {
      int best = -1;
      double best_d = cfg_.assoc_gate;
      for (size_t i = 0; i < tracks_.size(); ++i) {
        if (used[i]) {
          continue;
        }
        const double d = std::hypot(tracks_[i].px_ - c.x, tracks_[i].py_ - c.y);
        if (d < best_d) {
          best_d = d;
          best = static_cast<int>(i);
        }
      }
      if (best >= 0) {
        auto & t = tracks_[static_cast<size_t>(best)];
        // Observed motion of the centroid in the sensor frame, measured from the
        // previous MEASURED position (t.x/t.y), over the full dt.
        const double raw_vx = (c.x - t.x) / dt;
        const double raw_vy = (c.y - t.y) / dt;
        // Add back the sensor frame's own motion to get the obstacle's world
        // velocity. This is what makes a static wall read zero.
        double own_vx = 0.0, own_vy = 0.0;
        sensorFrameMotion(c.x, c.y, robot_vx, robot_vy, robot_wz, own_vx, own_vy);
        double evx = raw_vx + own_vx;
        double evy = raw_vy + own_vy;
        if (std::hypot(evx, evy) > cfg_.max_speed) {
          evx = t.vx;   // implausible: the association jumped surfaces
          evy = t.vy;
        }
        const double alpha = std::clamp(cfg_.vel_alpha, 0.0, 1.0);
        t.vx += alpha * (evx - t.vx);
        t.vy += alpha * (evy - t.vy);
        t.x = c.x;
        t.y = c.y;
        t.radius = 0.5 * c.width_m;
        t.points = c.points;
        t.missed = 0;
        t.measured = true;
        used[static_cast<size_t>(best)] = true;
      } else if (static_cast<int>(tracks_.size()) < cfg_.max_tracks) {
        ObstacleTrack t;
        t.id = next_id_++;
        t.x = c.x;
        t.y = c.y;
        t.vx = t.vy = 0.0;
        t.px_ = c.x;
        t.py_ = c.y;
        t.radius = 0.5 * c.width_m;
        t.points = c.points;
        t.age_s = 0.0;
        t.measured = true;
        tracks_.push_back(t);
        used.push_back(true);
      }
    }

    for (auto & t : tracks_) {
      if (!t.measured) {
        ++t.missed;
        // carry the prediction forward so a coasting track keeps moving
        t.x = t.px_;
        t.y = t.py_;
      }
    }
    tracks_.erase(
      std::remove_if(
        tracks_.begin(), tracks_.end(),
        [this](const ObstacleTrack & t) {return t.missed > cfg_.max_missed;}),
      tracks_.end());

    prev_vx_ = robot_vx;
    prev_vy_ = robot_vy;
    have_prev_twist_ = true;
  }

  /**
   * @brief Does anything enter the robot's swept path before it gets there?
   *
   * @param cmd_vx,cmd_vy  the intended body velocity (m/s)
   * @param exclude_x,exclude_y,exclude_r  the followed person, who is not an
   *        obstacle (pass exclude_r <= 0 to disable)
   *
   * THE MODEL. Both the robot and the obstacle are extrapolated at constant
   * velocity and the robot is asked: at the moment my leading edge reaches your
   * along-track position, is your lateral offset inside my half-width? The
   * obstacle's motion RELATIVE TO THE ROBOT is what decides that, so the robot's
   * own velocity is subtracted from the obstacle's world velocity. Getting this
   * wrong is subtle and was a real bug here: using the world velocity alone
   * makes a static wall's predicted position stand still in the sensor frame
   * while the robot drives at it, so nothing ever appeared to enter the path.
   *
   * `closing` is reported as a positive rate when the gap is shrinking, which is
   * what the TTC law needs. It is the full relative closing rate, so a pedestrian
   * walking INTO the robot at 0.4 m/s while the robot advances at 0.9 m/s reads
   * 1.3 m/s and the governor budgets for the room that actually requires.
   */
  CrossingResult crossingCheck(double cmd_vx, double cmd_vy,
                               double exclude_x, double exclude_y,
                               double exclude_r) const
  {
    CrossingResult out;
    const double v = std::hypot(cmd_vx, cmd_vy);
    if (v < 1e-3 || !cfg_.enable) {
      return out;
    }
    const double cd = cmd_vx / v;
    const double sd = cmd_vy / v;
    // half-width of the body perpendicular to travel + safety margin
    const double support_perp =
      0.5 * (cfg_.robot_length * std::abs(sd) + cfg_.robot_width * std::abs(cd));
    const double corridor = support_perp + cfg_.corridor_margin;
    const double support_travel =
      0.5 * (cfg_.robot_length * std::abs(cd) + cfg_.robot_width * std::abs(sd));

    double best_t = std::numeric_limits<double>::infinity();
    for (const auto & t : tracks_) {
      if (exclude_r > 0.0 &&
        std::hypot(t.x - exclude_x, t.y - exclude_y) < exclude_r)
      {
        continue;
      }
      // The obstacle's velocity relative to the robot, in the sensor frame. The
      // robot advances at (cmd_vx, cmd_vy), so subtracting it leaves the motion
      // that closes the gap.
      const double rvx = t.vx - cmd_vx;
      const double rvy = t.vy - cmd_vy;

      // Time for the robot's leading edge to reach the obstacle's along-track
      // position, solved iteratively because the obstacle keeps moving while the
      // robot approaches. Each step re-evaluates the along-track distance at the
      // predicted position; three passes is ample for the velocities involved.
      double t_hit = std::numeric_limits<double>::infinity();
      double ox = t.x, oy = t.y;
      for (int iter = 0; iter < 4; ++iter) {
        const double along = ox * cd + oy * sd;
        if (along <= 0.0) {
          t_hit = std::numeric_limits<double>::infinity();
          break;
        }
        const double room = std::max(0.0, along - support_travel);
        t_hit = room / v;
        if (!std::isfinite(t_hit) || t_hit > cfg_.lookahead) {
          t_hit = std::numeric_limits<double>::infinity();
          break;
        }
        ox = t.x + rvx * t_hit;
        oy = t.y + rvy * t_hit;
      }
      if (!std::isfinite(t_hit)) {
        continue;
      }

      // Where is it laterally when the robot arrives?
      const double lat = -ox * sd + oy * cd;
      const double along = ox * cd + oy * sd;
      // distance from the body rectangle to the obstacle's centre
      const double clr = std::hypot(std::max(0.0, along - support_travel),
                                    std::max(0.0, std::abs(lat) - support_perp)) -
        t.radius;
      if (std::abs(lat) <= corridor + t.radius) {
        if (t_hit < best_t) {
          best_t = t_hit;
          out.crossing = true;
          out.t_arrive = t_hit;
          out.clearance = clr;
          out.id = t.id;
          out.ox = ox;
          out.oy = oy;
          // Closing rate along the line of sight: how fast the gap between the
          // body and this obstacle is shrinking right now.
          const double rng = std::max(1e-3, std::hypot(t.x, t.y));
          const double ux = t.x / rng, uy = t.y / rng;
          out.closing = std::max(0.0, -(rvx * ux + rvy * uy));
        }
      }
    }
    return out;
  }

  /**
   * @brief Largest closing rate toward the robot among all tracks.
   *
   * Used for the TTC law: the governor's plain curve assumes a static obstacle,
   * so anything moving toward the robot must have its own speed added to the
   * closing rate or the robot budgets for half the room it actually needs.
   */
  double maxClosingRate() const
  {
    double c = 0.0;
    for (const auto & t : tracks_) {
      const double rng = std::hypot(t.x, t.y);
      if (rng < 1e-3) {
        continue;
      }
      const double ux = t.x / rng, uy = t.y / rng;
      c = std::max(c, -(t.vx * ux + t.vy * uy));
    }
    return c;
  }

  /**
   * @brief Nearest moving obstacle to the robot, or nullptr.
   * @param min_speed only consider obstacles moving at least this fast (m/s)
   */
  const ObstacleTrack * nearestMoving(double min_speed = 0.15) const
  {
    const ObstacleTrack * best = nullptr;
    double best_r = std::numeric_limits<double>::infinity();
    for (const auto & t : tracks_) {
      if (!t.moving(min_speed)) {
        continue;
      }
      const double r = t.range();
      if (r < best_r) {
        best_r = r;
        best = &t;
      }
    }
    return best;
  }

private:
  /**
   * @brief How fast the SENSOR FRAME moves under a point at (x, y).
   *
   * The sensor frame is rigidly attached to the robot, so a point fixed in the
   * world appears to move in the sensor frame at
   *     own_v = (vx_b - wz * y,  vy_b + wz * x)
   * which is the body velocity plus the lever arm of the yaw rate. Subtracting
   * this from the observed motion gives the obstacle's true world velocity; a
   * static wall therefore reads exactly zero.
   */
  static void sensorFrameMotion(double x, double y,
                                double vx_b, double vy_b, double wz,
                                double & own_vx, double & own_vy)
  {
    own_vx = vx_b - wz * y;
    own_vy = vy_b + wz * x;
  }

  struct Cluster
  {
    double x = 0.0;
    double y = 0.0;
    double width_m = 0.0;
    int points = 0;
  };

  std::vector<Cluster> extractClusters(const ScanFrame & scan) const
  {
    std::vector<Cluster> out;
    const int n = static_cast<int>(scan.ranges.size());
    if (n <= 0) {
      return out;
    }
    const double bin_ang = std::abs(scan.angle_increment);

    const auto selfOccluded = [&](int idx) {
        const double r = scan.obstacleAt(idx);
        if (!std::isfinite(r)) {
          return false;
        }
        const double a = scan.angleAt(idx);
        const double px = r * std::cos(a);
        const double py = r * std::sin(a);
        return (px > -cfg_.frame_back && px < cfg_.frame_front &&
               py > -cfg_.frame_right && py < cfg_.frame_left);
      };

    int i = 0;
    while (i < n) {
      const double r0 = scan.obstacleAt(i);
      if (!std::isfinite(r0) || r0 > cfg_.max_range || selfOccluded(i)) {
        ++i;
        continue;
      }
      int j = i;
      int last_populated = i;
      int used = 0;
      double sx = 0.0, sy = 0.0;
      double min_r = std::numeric_limits<double>::infinity();
      while (j < n && (j - last_populated) <= cfg_.max_gap_bins) {
        const double r = scan.obstacleAt(j);
        const double a = scan.angleAt(j);
        const bool usable = std::isfinite(r) && r <= cfg_.max_range && !selfOccluded(j);
        if (usable) {
          sx += r * std::cos(a);
          sy += r * std::sin(a);
          min_r = std::min(min_r, r);
          ++used;
          last_populated = j;
        }
        ++j;
      }
      if (used >= cfg_.min_points && std::isfinite(min_r)) {
        const double a0 = scan.angleAt(i);
        const double a1 = scan.angleAt(last_populated);
        const double span = std::abs(a1 - a0) + bin_ang;
        const double width_m = 2.0 * min_r * std::sin(std::min(span, M_PI) / 2.0);
        if (width_m >= cfg_.min_width && width_m <= cfg_.max_width) {
          Cluster c;
          c.x = sx / used;
          c.y = sy / used;
          c.width_m = width_m;
          c.points = used;
          out.push_back(c);
        }
      }
      i = std::max(j, i + 1);
    }
    return out;
  }

  DynamicObstacleConfig cfg_;
  std::vector<ObstacleTrack> tracks_;
  int next_id_ = 0;
  double prev_vx_ = 0.0, prev_vy_ = 0.0;
  bool have_prev_twist_ = false;
};

}  // namespace rs_follow

#endif  // RS_FOLLOW_CORE_DYNAMIC_OBSTACLE_HPP
