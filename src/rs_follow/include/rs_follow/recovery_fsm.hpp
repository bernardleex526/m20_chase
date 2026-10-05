/**
 * @file recovery_fsm.hpp
 * @brief Two small state machines that turn "stuck" into "recovers".
 *
 * The local planner (VFH+) is reactive, so it has two well-known failure modes
 * that were measured directly before this existed:
 *
 *   1. DEADLOCK -- the robot is commanded to move but does not move (blocked,
 *      wedged in a U, or creeping against something). Observed as
 *      EMERGENCY_STOP for 10.8 s in the corridor scenario and permanent
 *      NO_TARGET stalls in the trap scenario.
 *   2. TARGET LOSS -- the person is occluded for longer than the tracker's
 *      timeout, the target is cleared, and nothing ever re-acquires it. The
 *      stock behaviour coasts ~3 s and then sits still forever.
 *
 * Both are handled here rather than in the controller, so the control law stays
 * a pure function of the current scan.
 *
 * The recovery machine deliberately has a terminal HOLD state: retrying a
 * blocked path forever is more dangerous than stopping and reporting.
 */

#ifndef RS_FOLLOW_RECOVERY_FSM_HPP
#define RS_FOLLOW_RECOVERY_FSM_HPP

#include <algorithm>
#include <cmath>
#include <string>

namespace rs_follow
{

enum class RecoverState { FOLLOW, BACKUP, SPIN, HOLD };
enum class SearchState { TRACK, COAST, SPIN_SCAN, GO_LAST, HOLD };

inline const char * toString(RecoverState s)
{
  switch (s) {
    case RecoverState::FOLLOW: return "FOLLOW";
    case RecoverState::BACKUP: return "BACKUP";
    case RecoverState::SPIN:   return "SPIN";
    case RecoverState::HOLD:   return "HOLD";
  }
  return "?";
}

inline const char * toString(SearchState s)
{
  switch (s) {
    case SearchState::TRACK:     return "TRACK";
    case SearchState::COAST:     return "COAST";
    case SearchState::SPIN_SCAN: return "SPIN_SCAN";
    case SearchState::GO_LAST:   return "GO_LAST";
    case SearchState::HOLD:      return "HOLD";
  }
  return "?";
}

struct RecoveryConfig
{
  bool enable = true;
  // "Not making progress" test: commanded to move, but the measured pose barely
  // changed over this window.
  double stuck_window = 2.0;      // s
  double stuck_dist = 0.05;       // m of travel below which we call it stuck
  double stuck_min_cmd = 0.10;    // only if we were actually asked to move
  double backup_dist = 0.40;      // m to reverse
  double backup_speed = 0.25;     // m/s
  double spin_rate = 0.60;        // rad/s while searching for a way out
  double spin_timeout = 6.0;      // s per spin attempt
  int max_attempts = 3;           // attempts before holding
  // HOLD is not terminal. A HOLD that can never be left is a dead robot: the
  // machine latched it after 3 failed attempts and every later scenario then
  // commanded zero forever (the crowd scenario never moved at all). After this
  // long the world may have changed, so retry with a clean slate. The machine
  // still stops the robot while holding, which is the safe behaviour.
  double hold_timeout = 10.0;
};

struct SearchConfig
{
  bool enable = true;
  double coast_time = 1.0;        // s of inertial coast after the last sighting
  double spin_timeout = 12.0;     // s per scanning sweep
  double go_speed = 0.35;         // m/s while travelling to the last known spot
  double go_tol = 0.5;            // m considered "arrived"
  double go_timeout = 12.0;       // s per go-to attempt
  // Total time the search may keep trying before it HOLDS. A search that gives
  // up while the person is still in the world is not a search: the earlier
  // version stopped after a single 12 s sweep and one go-to attempt, and the
  // person walked back in a second later with the robot already parked.
  double search_budget = 60.0;
  int max_sweeps = 6;
};

/**
 * @brief Detects lack of progress and sequences BACKUP / SPIN / HOLD.
 *
 * The caller feeds the measured pose and the commanded velocity; the machine
 * returns a velocity override when it is not in FOLLOW.
 */
class RecoveryMachine
{
public:
  void setConfig(const RecoveryConfig & c) {cfg_ = c;}
  const RecoveryConfig & config() const {return cfg_;}
  RecoverState state() const {return st_;}
  int attempts() const {return attempts_;}

  void reset()
  {
    st_ = RecoverState::FOLLOW;
    attempts_ = 0;
    win_t0_ = -1.0;
    win_x_ = win_y_ = 0.0;
    phase_t0_ = 0.0;
    spin_dir_ = 1.0;
  }

  /** @return true if the machine wants to override the controller's command. */
  bool update(double t, double x, double y, double yaw,
              double cmd_vx, double cmd_vy, double cmd_wz,
              double rear_clear, double* out_vx, double* out_vy, double* out_wz)
  {
    (void)yaw;
    (void)cmd_wz;
    *out_vx = *out_vy = *out_wz = 0.0;
    if (!cfg_.enable) {
      return false;
    }

    switch (st_) {
      case RecoverState::FOLLOW: {
        const bool commanded = std::hypot(cmd_vx, cmd_vy) > cfg_.stuck_min_cmd;
        if (!commanded) {
          win_t0_ = -1.0;
          return false;
        }
        if (win_t0_ < 0.0) {
          win_t0_ = t;
          win_x_ = x;
          win_y_ = y;
          return false;
        }
        const double moved = std::hypot(x - win_x_, y - win_y_);
        if (moved > cfg_.stuck_dist) {
          win_t0_ = t;      // progress: restart the window
          win_x_ = x;
          win_y_ = y;
          return false;
        }
        if (t - win_t0_ < cfg_.stuck_window) {
          return false;
        }
        // --- stuck ---
        if (attempts_ >= cfg_.max_attempts) {
          st_ = RecoverState::HOLD;
          return true;
        }
        ++attempts_;
        phase_t0_ = t;
        // Prefer reversing when there is room behind; otherwise spin in place.
        st_ = (rear_clear > (cfg_.backup_dist + 0.15)) ? RecoverState::BACKUP
                                                       : RecoverState::SPIN;
        return true;
      }

      case RecoverState::BACKUP: {
        const double el = t - phase_t0_;
        const double travelled = cfg_.backup_speed * el;
        if (travelled >= cfg_.backup_dist || el > (cfg_.backup_dist / cfg_.backup_speed) * 1.6) {
          st_ = RecoverState::SPIN;
          phase_t0_ = t;
          return true;
        }
        *out_vx = -cfg_.backup_speed;
        return true;
      }

      case RecoverState::SPIN: {
        const double el = t - phase_t0_;
        if (el > cfg_.spin_timeout) {
          // give the normal controller another go
          st_ = RecoverState::FOLLOW;
          win_t0_ = -1.0;
          return false;
        }
        *out_wz = spin_dir_ * cfg_.spin_rate;
        return true;
      }

      case RecoverState::HOLD:
        if (t - phase_t0_ > cfg_.hold_timeout) {
          // retry from scratch: the obstacle or the person may have moved
          st_ = RecoverState::FOLLOW;
          attempts_ = 0;
          win_t0_ = -1.0;
          return false;
        }
        return true;
    }
    return false;
  }

  /** Let the caller flip the spin direction (e.g. toward the freer side). */
  void setSpinDirection(double d) {spin_dir_ = (d >= 0.0) ? 1.0 : -1.0;}

private:
  RecoveryConfig cfg_;
  RecoverState st_ = RecoverState::FOLLOW;
  int attempts_ = 0;
  double win_t0_ = -1.0, win_x_ = 0.0, win_y_ = 0.0;
  double phase_t0_ = 0.0;
  double spin_dir_ = 1.0;
};

/**
 * @brief Re-acquires a person after the tracker has given up on them.
 *
 * Sequence: TRACK -> (tracker reports loss) COAST -> SPIN_SCAN -> GO_LAST ->
 * HOLD. Every phase can be pre-empted by the tracker re-acquiring the target,
 * which is what makes it recover instead of sitting still forever.
 */
class TargetSearchMachine
{
public:
  void setConfig(const SearchConfig & c) {cfg_ = c;}
  const SearchConfig & config() const {return cfg_;}
  SearchState state() const {return st_;}
  bool holding() const {return st_ == SearchState::HOLD;}

  void reset()
  {
    st_ = SearchState::TRACK;
    phase_t0_ = 0.0;
    have_last_ = false;
    search_t0_ = 0.0;
    sweeps_ = 0;
  }

  /** Record the last place the person was seen (world frame). */
  void noteSighting(double t, double x, double y)
  {
    last_x_ = x;
    last_y_ = y;
    have_last_ = true;
    last_seen_t_ = t;
    if (st_ != SearchState::TRACK) {
      st_ = SearchState::TRACK;
      sweeps_ = 0;
    }
  }

  /** Call when the tracker has no target. */
  void noteLost(double t)
  {
    if (st_ == SearchState::TRACK) {
      st_ = SearchState::COAST;
      phase_t0_ = t;
      search_t0_ = t;
      sweeps_ = 0;
    }
  }

  /**
   * @return true if this machine wants to override the command.
   * @param spin_out  angular velocity for the scan phase
   * @param vx_out    linear velocity for the go-to phase
   */
  bool update(double t, double x, double y,
              double* vx_out, double* wz_out, bool* arrived)
  {
    *vx_out = 0.0;
    *wz_out = 0.0;
    *arrived = false;
    if (!cfg_.enable) {
      return false;
    }

    switch (st_) {
      case SearchState::TRACK:
        return false;

      case SearchState::COAST:
        // let the tracker's own coasting finish; do not command anything yet
        if (t - phase_t0_ > cfg_.coast_time) {
          st_ = SearchState::SPIN_SCAN;
          phase_t0_ = t;
        }
        return false;

      case SearchState::SPIN_SCAN:
        // one full sweep done: try the last known spot, or sweep again
        if (t - phase_t0_ > cfg_.spin_timeout) {
          ++sweeps_;
          phase_t0_ = t;
          if (!have_last_ || sweeps_ >= cfg_.max_sweeps) {
            st_ = SearchState::HOLD;
            return false;
          }
          st_ = SearchState::GO_LAST;
          return true;
        }
        *wz_out = 0.6;
        return true;

      case SearchState::GO_LAST: {
        if (t - search_t0_ > cfg_.search_budget) {
          st_ = SearchState::HOLD;
          return false;
        }
        if (!have_last_) {
          st_ = SearchState::SPIN_SCAN;
          phase_t0_ = t;
          return false;
        }
        const double d = std::hypot(last_x_ - x, last_y_ - y);
        // Already standing where the person was last seen (the usual case: the
        // robot was following them at a 1 m standoff). Driving to that spot
        // achieves nothing, so sweep again instead of declaring the search over.
        if (d < cfg_.go_tol || t - phase_t0_ > cfg_.go_timeout) {
          *arrived = true;
          st_ = SearchState::SPIN_SCAN;
          phase_t0_ = t;
          return false;
        }
        // steer toward the last known spot, creep forward
        const double want = std::atan2(last_y_ - y, last_x_ - x);
        *wz_out = std::clamp(1.2 * want, -1.0, 1.0);
        *vx_out = cfg_.go_speed;
        return true;
      }

      case SearchState::HOLD:
        return false;
    }
    return false;
  }

  double lastX() const {return last_x_;}
  double lastY() const {return last_y_;}

private:
  SearchConfig cfg_;
  SearchState st_ = SearchState::TRACK;
  double phase_t0_ = 0.0;
  double last_x_ = 0.0, last_y_ = 0.0;
  double last_seen_t_ = 0.0;
  double search_t0_ = 0.0;
  int sweeps_ = 0;
  bool have_last_ = false;
};

}  // namespace rs_follow

#endif  // RS_FOLLOW_RECOVERY_FSM_HPP