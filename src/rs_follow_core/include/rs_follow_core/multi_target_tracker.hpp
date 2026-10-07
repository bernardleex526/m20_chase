/**
 * @file multi_target_tracker.hpp
 * @brief Extract several person-like clusters from one scan and keep IDs on them.
 *
 * WHY THIS EXISTS
 * ---------------
 * The stock tracker holds exactly one target. That is enough to walk behind a
 * single person, but it has three consequences the evaluation called out:
 *
 *   * In a crowd it can only ever see "the one I locked"; a second person
 *     walking between the robot and the target is indistinguishable from the
 *     target being occluded, so the lock is either held by dead reckoning or
 *     dropped and re-acquired on the WRONG person.
 *   * Target loss ends in a blind re-acquisition: the robot takes the nearest
 *     person-like cluster in front of it, which is not necessarily the person it
 *     was following.
 *   * There is nowhere to hang a confidence or an identity, so "is this still
 *     the same person" cannot be asked at all.
 *
 * This module turns the scan into a set of candidate people, each with a stable
 * integer id, a Kalman-filtered position/velocity and a confidence. It does NOT
 * do appearance Re-ID (that needs a camera and a learned descriptor, and this
 * library is deliberately dependency-free) -- but it makes the geometry explicit
 * so the follow controller can reason about identity over time:
 *
 *   * a track that is not measured this frame is PREDICTED, not deleted, so a
 *     person who is briefly occluded keeps their id and their estimated motion;
 *   * when they reappear, association is by predicted position, so the robot
 *     resumes on the same id rather than on whoever happens to be closest;
 *   * a second person entering the scene becomes a NEW id, and never steals the
 *     locked one, because selection is by id first and only by heuristic when
 *     nothing is locked.
 *
 * Clustering is the same geometry the single-target auto-select already uses
 * (gap-bridged runs, self-occlusion exclusion, physical-width test), extended to
 * the full circle and made range-independent, because that is what separates a
 * person from a wall without a learned model.
 */

#ifndef RS_FOLLOW_CORE_MULTI_TARGET_TRACKER_HPP
#define RS_FOLLOW_CORE_MULTI_TARGET_TRACKER_HPP

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "rs_follow_core/kalman_filter_2d.hpp"
#include "rs_follow_core/pointcloud_scan.hpp"

namespace rs_follow
{

/** @brief One candidate person, sensor frame. */
struct TargetTrack
{
  int id = -1;
  double x = 0.0;              // filtered position (m), sensor frame
  double y = 0.0;
  double raw_x = 0.0;          // last measured centroid (m)
  double raw_y = 0.0;
  double vx = 0.0;             // filtered velocity (m/s), sensor frame
  double vy = 0.0;
  double range = 0.0;          // hypot(x, y)
  double bearing = 0.0;        // atan2(y, x)
  double width_m = 0.0;        // physical width of the visible arc (m)
  int points = 0;              // populated bins in the cluster
  int missed = 0;              // frames since last measurement
  double confidence = 1.0;     // 0..1, decays while coasting
  double age_s = 0.0;          // seconds since the track was created
  bool measured = false;       // had a real measurement this frame
  // Per-track constant-velocity filter. Kept per track so a crowd of people is
  // filtered independently; a shared filter would blend their motion together.
  KalmanFilter2D kf;

  bool alive() const {return id >= 0;}
};

struct MultiTargetConfig
{
  bool enable = false;
  // --- clustering ---
  double max_range = 8.0;         // ignore returns beyond this (m)
  int max_gap_bins = 4;           // bridge holes up to this many bins
  int min_points = 2;             // populated bins needed to call it a cluster
  // Physical width (m) accepted as a person. A person is ~0.5 m across; a wall
  // reads far wider at the same range. This is range-independent by design: a
  // distance threshold would only work at one standoff.
  double min_width = 0.08;
  double max_width = 0.90;
  // --- association ---
  // Gate (m) for matching a measurement to an existing track's PREDICTED
  // position. Too small and a walking person is dropped and re-created every
  // frame (id churn); too large and two nearby people merge.
  double assoc_gate = 0.70;
  // --- track lifecycle ---
  int max_missed = 15;            // frames of prediction before a track dies
  int max_tracks = 8;             // hard cap on simultaneous tracks
  double confirm_age_s = 0.0;     // min age before a track is selectable
  // --- selection ---
  enum class Select
  {
    kNearest = 0,     // closest track (classic auto-select)
    kPersonLike,      // closest whose width is closest to a real person
    kIdLock,          // keep the currently locked id; else fall back
    kLargest,         // most points, i.e. the least occluded
  };
  Select select = Select::kIdLock;
  // Width of a real person (m), used by kPersonLike to rank candidates.
  double person_width = 0.50;
  // Prefer tracks that are already confirmed by this many measurements.
  int confirm_hits = 2;
  // Robot self-occlusion box (m), sensor frame. MUST match the controller's
  // outline: the chassis sits inside the target height band and is nearer than
  // any person, so if it is not excluded here the robot selects itself.
  double frame_front = 0.25;
  double frame_back = 0.45;
  double frame_left = 0.25;
  double frame_right = 0.25;
};

/**
 * @brief Cluster the scan into candidate people and maintain tracks across frames.
 *
 * The tracker owns no ROS types and no clock: `dt` is passed in, so a unit test
 * can drive it deterministically.
 */
class MultiTargetTracker
{
public:
  void setConfig(const MultiTargetConfig & cfg) {cfg_ = cfg;}
  const MultiTargetConfig & config() const {return cfg_;}

  const std::vector<TargetTrack> & tracks() const {return tracks_;}
  int lockedId() const {return locked_id_;}

  void reset()
  {
    tracks_.clear();
    locked_id_ = -1;
    next_id_ = 0;
    time_s_ = 0.0;
  }

  void clearLock() {locked_id_ = -1;}
  void lockId(int id) {locked_id_ = id;}

  /**
   * @brief Cluster the scan and update every track by one frame.
   * @param scan  polar scan (target band is what people are made of)
   * @param dt    seconds since the previous call
   */
  void update(const ScanFrame & scan, double dt)
  {
    if (!cfg_.enable) {
      return;
    }
    dt = std::clamp(dt, 0.001, 0.5);
    time_s_ += dt;

    std::vector<Cluster> clusters = extractClusters(scan);

    // --- predict every track forward ---
    for (auto & t : tracks_) {
      double fx = t.x, fy = t.y;
      t.kf.predictOnly(dt, fx, fy);
      t.x = fx;
      t.y = fy;
      t.vx = t.kf.getVelocityX();
      t.vy = t.kf.getVelocityY();
      t.measured = false;
      t.age_s += dt;
    }

    // --- associate each cluster with the nearest free track (greedy) ---
    std::vector<bool> track_used(tracks_.size(), false);
    for (const auto & c : clusters) {
      int best = -1;
      double best_d = cfg_.assoc_gate;
      for (size_t i = 0; i < tracks_.size(); ++i) {
        if (track_used[i]) {
          continue;
        }
        const double d = std::hypot(tracks_[i].x - c.x, tracks_[i].y - c.y);
        if (d < best_d) {
          best_d = d;
          best = static_cast<int>(i);
        }
      }
      if (best >= 0) {
        auto & t = tracks_[static_cast<size_t>(best)];
        double fx = 0.0, fy = 0.0;
        bool accepted = true;
        t.kf.update(c.x, c.y, fx, fy, dt, &accepted, c.points);
        if (accepted) {
          t.x = fx;
          t.y = fy;
          t.raw_x = c.x;
          t.raw_y = c.y;
          t.vx = t.kf.getVelocityX();
          t.vy = t.kf.getVelocityY();
          t.width_m = c.width_m;
          t.points = c.points;
          t.missed = 0;
          t.measured = true;
          t.confidence = std::min(1.0, t.confidence + 0.25);
        }
        track_used[static_cast<size_t>(best)] = true;
      } else if (static_cast<int>(tracks_.size()) < cfg_.max_tracks) {
        tracks_.push_back(makeTrack(c));
        track_used.push_back(true);
      }
    }

    // --- age out tracks that were not measured ---
    for (auto & t : tracks_) {
      if (!t.measured) {
        ++t.missed;
        // confidence decays with the prediction horizon: after max_missed
        // frames the position is extrapolation, not observation.
        t.confidence = std::max(
          0.0, 1.0 - static_cast<double>(t.missed) /
          std::max(1, cfg_.max_missed));
      }
      t.range = std::hypot(t.x, t.y);
      t.bearing = std::atan2(t.y, t.x);
    }
    tracks_.erase(
      std::remove_if(
        tracks_.begin(), tracks_.end(),
        [this](const TargetTrack & t) {return t.missed > cfg_.max_missed;}),
      tracks_.end());

    // A locked track that died must release the lock, or selection would keep
    // returning a stale id forever.
    if (locked_id_ >= 0 && findById(locked_id_) == nullptr) {
      locked_id_ = -1;
    }
  }

  /** @brief Index of the track with this id, or -1. */
  int indexOf(int id) const
  {
    for (size_t i = 0; i < tracks_.size(); ++i) {
      if (tracks_[i].id == id) {
        return static_cast<int>(i);
      }
    }
    return -1;
  }

  const TargetTrack * findById(int id) const
  {
    const int i = indexOf(id);
    return i >= 0 ? &tracks_[static_cast<size_t>(i)] : nullptr;
  }

  /**
   * @brief Choose the track to follow.
   *
   * With `Select::kIdLock` an existing lock is kept as long as the track is
   * alive -- this is the behaviour that stops a passer-by from stealing the
   * follow. Otherwise the configured heuristic runs, restricted to tracks that
   * are confirmed (enough measurements) and not stale.
   *
   * @param fov_half_rad  restrict candidates to this half-angle about straight
   *                      ahead; <= 0 or >= pi means "every direction"
   * @return index into tracks(), or -1
   */
  int select(double fov_half_rad = M_PI, double max_range = 0.0) const
  {
    if (tracks_.empty()) {
      return -1;
    }
    const double range_cap = max_range > 0.0 ? max_range : cfg_.max_range;

    if (cfg_.select == MultiTargetConfig::Select::kIdLock && locked_id_ >= 0) {
      const int i = indexOf(locked_id_);
      if (i >= 0) {
        return i;
      }
    }

    int best = -1;
    double best_score = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < tracks_.size(); ++i) {
      const auto & t = tracks_[i];
      if (t.range > range_cap) {
        continue;
      }
      if (fov_half_rad > 0.0 && fov_half_rad < M_PI &&
        std::abs(t.bearing) > fov_half_rad)
      {
        continue;
      }
      // A track seen only once is a guess; require confirmation unless nothing
      // else is available at all (handled by the second pass below).
      if (t.points < cfg_.min_points) {
        continue;
      }
      double score = t.range;
      switch (cfg_.select) {
        case MultiTargetConfig::Select::kNearest:
        case MultiTargetConfig::Select::kIdLock:
          score = t.range;
          break;
        case MultiTargetConfig::Select::kPersonLike:
          score = t.range + 2.0 * std::abs(t.width_m - cfg_.person_width);
          break;
        case MultiTargetConfig::Select::kLargest:
          score = -static_cast<double>(t.points);
          break;
      }
      if (score < best_score) {
        best_score = score;
        best = static_cast<int>(i);
      }
    }
    return best;
  }

  /** @brief Lock onto a track (e.g. after a manual bind). */
  void lockIndex(int i)
  {
    if (i >= 0 && i < static_cast<int>(tracks_.size())) {
      locked_id_ = tracks_[static_cast<size_t>(i)].id;
    }
  }

  /**
   * @brief Lock onto the track nearest a point (manual bind / click).
   * @return true when a track was found within `radius`
   */
  bool lockNearest(double x, double y, double radius)
  {
    int best = -1;
    double best_d = radius;
    for (size_t i = 0; i < tracks_.size(); ++i) {
      const double d = std::hypot(tracks_[i].x - x, tracks_[i].y - y);
      if (d < best_d) {
        best_d = d;
        best = static_cast<int>(i);
      }
    }
    if (best < 0) {
      return false;
    }
    locked_id_ = tracks_[static_cast<size_t>(best)].id;
    return true;
  }

private:
  /** @brief A raw cluster before it becomes a track. */
  struct Cluster
  {
    double x = 0.0;
    double y = 0.0;
    double width_m = 0.0;
    int points = 0;
  };

  TargetTrack makeTrack(const Cluster & c)
  {
    TargetTrack t;
    t.id = next_id_++;
    t.x = t.raw_x = c.x;
    t.y = t.raw_y = c.y;
    t.width_m = c.width_m;
    t.points = c.points;
    t.range = std::hypot(c.x, c.y);
    t.bearing = std::atan2(c.y, c.x);
    t.missed = 0;
    t.confidence = 0.5;
    t.age_s = 0.0;
    t.measured = true;
    t.kf.setProcessNoise(0.1);
    t.kf.setMeasurementNoise(0.05);
    t.kf.setGate(6.0);
    t.kf.setMeasurementNoiseScale(80);
    t.kf.setState(c.x, c.y);
    return t;
  }

  /**
   * @brief Group in-range bins into clusters and keep the person-like ones.
   *
   * Three details matter and each one was a real bug in the single-target
   * version:
   *   * HOLES. The projection has 1440 bins (0.25 deg) but a 720-ray LiDAR
   *     leaves every second bin empty, and the robot's own chassis cuts a
   *     contiguous arc out of the middle of the view. Requiring two ADJACENT
   *     populated bins therefore never holds -- a person was once cut into 47
   *     single-bin fragments and could never be acquired. Gaps up to
   *     `max_gap_bins` are bridged.
   *   * SELF-OCCLUSION. The chassis sits inside the target height band and is
   *     nearer than any person, so it must be excluded here too, or the robot
   *     selects itself.
   *   * WIDTH. A cluster's angular span converted to metres at its range is what
   *     separates a person (~0.5 m) from a wall (metres wide at any range).
   */
  std::vector<Cluster> extractClusters(const ScanFrame & scan) const
  {
    std::vector<Cluster> out;
    const int n = static_cast<int>(scan.ranges.size());
    if (n <= 0) {
      return out;
    }
    const double bin_ang = std::abs(scan.angle_increment);

    const auto selfOccluded = [&](int idx) {
        const double r = scan.ranges[static_cast<size_t>(idx)];
        const double a = scan.angleAt(idx);
        const double px = r * std::cos(a);
        const double py = r * std::sin(a);
        return (px > -cfg_.frame_back && px < cfg_.frame_front &&
               py > -cfg_.frame_right && py < cfg_.frame_left);
      };

    int i = 0;
    while (i < n) {
      const double r0 = scan.ranges[static_cast<size_t>(i)];
      if (!std::isfinite(r0) || r0 > cfg_.max_range || selfOccluded(i)) {
        ++i;
        continue;
      }
      // Grow a run, tolerating holes up to max_gap_bins.
      int j = i;
      int last_populated = i;
      int used = 0;
      double sx = 0.0, sy = 0.0;
      double min_r = std::numeric_limits<double>::infinity();
      while (j < n && (j - last_populated) <= cfg_.max_gap_bins) {
        const double r = scan.ranges[static_cast<size_t>(j)];
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

  MultiTargetConfig cfg_;
  std::vector<TargetTrack> tracks_;
  int locked_id_ = -1;
  int next_id_ = 0;
  double time_s_ = 0.0;
};

}  // namespace rs_follow

#endif  // RS_FOLLOW_CORE_MULTI_TARGET_TRACKER_HPP
