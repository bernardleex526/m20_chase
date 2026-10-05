/**
 * @file vfh_planner.hpp
 * @brief Gap-based local steering (VFH+ style) over the polar scan.
 *
 * WHY THIS EXISTS
 * ---------------
 * The stock controller steers straight at the target bearing and adds a weak
 * artificial potential field. That combination cannot go around anything: with
 * a static obstacle between robot and target it either nudges sideways slightly
 * or hard-stops, and it never picks a different heading. Measured behaviour
 * before this planner: a 0.6 m gap between two obstacles produced vx=0.014 with
 * the robot shoved sideways, and a plain wall produced a permanent stop.
 *
 * WHAT IT DOES
 * ------------
 * For every candidate heading it computes how far the robot could actually
 * travel before its BODY touches something, then steers into the cheapest
 * heading that is both wide enough and closest to the target bearing. This is
 * the VFH+ idea (Ulrich & Borenstein) applied directly to the polar range scan
 * we already build, so it needs no costmap, no grid and no new dependency.
 *
 *     clearance(a) = range(a) - support(a)
 *     free(theta)  = min over the angular window the body sweeps at theta
 *     traversable  = free(theta) >= d_safe
 *     chosen       = argmin |wrap(theta - target)| + hysteresis, among traversable
 *
 * `support(a)` is the half-width of the robot outline perpendicular to travel
 * direction `a`; using it (rather than a fixed circumscribed circle) is what
 * lets a 0.62 x 0.36 m body thread a 0.90 m doorway that a 0.36 m radius
 * approximation would wrongly reject.
 */

#ifndef RS_FOLLOW_VFH_PLANNER_HPP
#define RS_FOLLOW_VFH_PLANNER_HPP

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "rs_follow/pointcloud_scan.hpp"

namespace rs_follow
{

struct VfhConfig
{
  bool enable = true;
  int bins = 360;              // planning resolution (1 deg at 360)
  double d_safe = 0.30;        // clearance that makes a heading traversable (m)
  // Derive d_safe from the governor instead of hand-tuning it: it must be
  // strictly larger than the hard-stop floor or the planner aims into it.
  bool auto_d_safe = true;
  double d_safe_margin = 0.10; // extra room on top of the stop band (m)
  double goal_weight = 1.0;    // cost per radian of deviation from the target
  double hysteresis_deg = 10.0;  // bonus (deg) for keeping the previous heading
  // Candidate headings are limited to +-this from straight ahead. 180 deg =
  // every direction is considered.
  //
  // This MUST stay at 180 for a follower. The cost function already prefers the
  // heading closest to the person, so there is nothing to protect against; but
  // clamping it made a person standing BEHIND the robot unreachable, because no
  // candidate heading pointed at them and the robot pirouetted between the
  // clamp limits forever (measured as bearing -152 deg with wz pinned at -1.00
  // in the side-wall and crowd scenarios, and the 1.0 m standoff never closing).
  double max_turn_deg = 180.0;
  // Clearance assigned to a bearing with no return at all.
  double free_max = 50.0;
};

struct VfhResult
{
  double direction = 0.0;      // chosen heading, sensor frame (rad)
  double free = 0.0;           // clearance along it (m)
  bool traversable = false;
  double best_free = 0.0;      // best clearance available anywhere (m)
  double best_dir = 0.0;       // heading achieving it
  bool blocked = false;        // no traversable heading at all
};

inline double wrapPi(double a)
{
  while (a > M_PI) {a -= 2.0 * M_PI;}
  while (a < -M_PI) {a += 2.0 * M_PI;}
  return a;
}

class VfhPlanner
{
public:
  void setConfig(const VfhConfig & cfg) {cfg_ = cfg;}
  const VfhConfig & config() const {return cfg_;}

  /**
   * @param scan             polar scan (target band + optional low band)
   * @param target_bearing   desired heading, sensor frame (rad)
   * @param prev_dir         previous chosen heading (for hysteresis)
   * @param support          callable: support(a) -> half-width (m)
   */
  template<typename SupportFn>
  VfhResult plan(const ScanFrame & scan, double target_bearing,
                 double prev_dir, SupportFn support)
  {
    VfhResult out;
    const int n = std::max(8, cfg_.bins);
    const int nbins_scan = static_cast<int>(scan.ranges.size());
    if (nbins_scan <= 0) {
      out.direction = target_bearing;
      out.free = cfg_.free_max;
      out.best_free = cfg_.free_max;
      out.best_dir = target_bearing;
      out.traversable = true;
      return out;
    }

    // --- 1. clearance profile at planning resolution ---
    std::vector<double> clr(static_cast<size_t>(n), cfg_.free_max);
    std::vector<double> sup(static_cast<size_t>(n), 0.0);
    std::vector<double> rng(static_cast<size_t>(n), cfg_.free_max);
    for (int k = 0; k < n; ++k) {
      const double a = -M_PI + (2.0 * M_PI) * (k + 0.5) / n;
      const double s = support(a);
      sup[static_cast<size_t>(k)] = s;
      // nearest scan bin
      int si = static_cast<int>((a - scan.angle_min) / scan.angle_increment);
      if (si < 0) {si = 0;}
      if (si >= nbins_scan) {si = nbins_scan - 1;}
      const float r = scan.obstacleAt(si);
      if (std::isfinite(r)) {
        rng[static_cast<size_t>(k)] = r;
        clr[static_cast<size_t>(k)] = std::max(0.0, static_cast<double>(r) - s);
      }
    }

    // --- 2. OBSTACLE ENLARGEMENT (the step that makes VFH+ work) ---
    //
    // A return at range r and bearing a blocks not just its own bin but every
    // heading whose body would sweep into it: half-angle asin(support/r). The
    // nearer the return, the wider it blocks.
    //
    // Doing this per OCCUPIED bin is essential. An earlier version instead
    // computed a windowed minimum around each candidate heading, sized from
    // that heading's own range -- which is self-defeating: a heading with no
    // return has range = free_max, so its window collapsed to a single bin and
    // the heading looked free even when a 0.5 m wide step sat 0.6 m ahead. The
    // robot would aim 1 deg off the step's corner and stall against it, which is
    // exactly the S8 failure (clearance pinned at 0.24 m, vz ~ 0.01 rad/s).
    std::vector<double> block_lo(static_cast<size_t>(n), 0.0);   // rad, relative
    std::vector<double> block_hi(static_cast<size_t>(n), 0.0);
    std::vector<double> free_d(static_cast<size_t>(n), cfg_.free_max);
    for (int k = 0; k < n; ++k) {
      if (rng[static_cast<size_t>(k)] >= cfg_.free_max) {
        continue;                                   // no return in this bin
      }
      const double r = std::max(rng[static_cast<size_t>(k)], 0.05);
      const double ratio = std::min(0.999, sup[static_cast<size_t>(k)] / r);
      const double half = std::asin(ratio);         // angular half-width blocked
      const int w = std::max(1, static_cast<int>(std::ceil(half / (2.0 * M_PI) * n)));
      for (int d = -w; d <= w; ++d) {
        int j = (k + d) % n;
        if (j < 0) {j += n;}
        // a blocked heading can only travel up to this return's range
        free_d[static_cast<size_t>(j)] =
          std::min(free_d[static_cast<size_t>(j)], r);
      }
    }

    // --- 3. convert the enlarged histogram into clearance per heading ---
    for (int k = 0; k < n; ++k) {
      free_d[static_cast<size_t>(k)] =
        std::max(0.0, free_d[static_cast<size_t>(k)] - sup[static_cast<size_t>(k)]);
    }

    // --- 4. choose the cheapest traversable heading ---
    const double hyst = cfg_.hysteresis_deg * M_PI / 180.0;
    const double max_turn = cfg_.max_turn_deg * M_PI / 180.0;
    double best_cost = std::numeric_limits<double>::infinity();
    double best_free_any = -1.0;
    double best_dir_any = target_bearing;
    bool found = false;

    for (int k = 0; k < n; ++k) {
      const double a = -M_PI + (2.0 * M_PI) * (k + 0.5) / n;
      const double f = free_d[static_cast<size_t>(k)];
      if (f > best_free_any) {
        best_free_any = f;
        best_dir_any = a;
      }
      if (std::abs(wrapPi(a)) > max_turn) {
        continue;  // do not solve a blocked path by reversing into the target
      }
      if (f < cfg_.d_safe) {
        continue;
      }
      const double dev = std::abs(wrapPi(a - target_bearing));
      double cost = cfg_.goal_weight * dev;
      // reward staying near the previous heading, so the choice does not chatter
      cost += cfg_.goal_weight * 0.5 * std::abs(wrapPi(a - prev_dir)) - hyst * 0.0;
      if (std::abs(wrapPi(a - prev_dir)) < hyst) {
        cost -= 0.15;
      }
      if (cost < best_cost) {
        best_cost = cost;
        out.direction = a;
        out.free = f;
        out.traversable = true;
        found = true;
      }
    }

    out.best_free = best_free_any;
    out.best_dir = best_dir_any;
    if (!found) {
      // Nothing is safely traversable: report the widest heading so the caller
      // can creep toward it, and let the speed governor decide the speed.
      out.direction = best_dir_any;
      out.free = std::max(0.0, best_free_any);
      out.traversable = false;
      out.blocked = true;
    }
    return out;
  }

private:
  VfhConfig cfg_;
};

}  // namespace rs_follow

#endif  // RS_FOLLOW_VFH_PLANNER_HPP