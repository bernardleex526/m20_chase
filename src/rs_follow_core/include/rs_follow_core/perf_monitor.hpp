/**
 * @file perf_monitor.hpp
 * @brief Lightweight stage timing and resource accounting for the control path.
 *
 * WHY THIS EXISTS
 * ---------------
 * The evaluation report could not answer "how much CPU does this use" or "which
 * stage is the bottleneck": every stage was silent, so the only available
 * measurement was external (a `ps` sample of the whole process). A control loop
 * that runs at 50 Hz has a hard 20 ms budget, and without per-stage numbers
 * there is no way to tell a projection problem from a planner problem.
 *
 * This is deliberately dependency-free: a steady_clock, a fixed-size table of
 * stages, and running statistics. It is header-only and adds no allocation on
 * the hot path.
 *
 * Typical use:
 *     PerfMonitor perf;
 *     {PerfScope s(perf, Stage::kProjection); projectPointCloud(...); }
 *     ...
 *     perf.report();          // one line per stage, human readable
 *
 * The scope guard costs two clock reads per call, which is negligible against a
 * 1440-bin projection but is NOT free. Keep `enable` false on a hard real-time
 * deployment if the numbers are not being collected.
 */

#ifndef RS_FOLLOW_CORE_PERF_MONITOR_HPP
#define RS_FOLLOW_CORE_PERF_MONITOR_HPP

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <string>

#if defined(__linux__)
#include <cstdio>
#endif

namespace rs_follow
{

/** @brief Named stages of the control path. Add new ones before kCount. */
enum class Stage : int
{
  kProjection = 0,   // 3D cloud -> polar scan
  kController,       // tracking + control law + safety
  kSmoothing,        // output limiting
  kPublish,          // serialisation + publish
  kTotal,            // whole callback / control tick
  kCount
};

inline const char * toString(Stage s)
{
  switch (s) {
    case Stage::kProjection: return "projection";
    case Stage::kController: return "controller";
    case Stage::kSmoothing:  return "smoothing";
    case Stage::kPublish:    return "publish";
    case Stage::kTotal:      return "total";
    case Stage::kCount:      break;
  }
  return "?";
}

/** @brief Running statistics for one stage. */
struct StageStats
{
  uint64_t count = 0;
  double total_ms = 0.0;
  double min_ms = 0.0;
  double max_ms = 0.0;
  // Last N samples, used for a percentile without keeping the full history.
  static constexpr int kWindow = 256;
  std::array<float, kWindow> window{};
  int window_pos = 0;
  int window_fill = 0;

  void add(double ms)
  {
    if (count == 0) {
      min_ms = max_ms = ms;
    } else {
      min_ms = std::min(min_ms, ms);
      max_ms = std::max(max_ms, ms);
    }
    total_ms += ms;
    ++count;
    window[static_cast<size_t>(window_pos)] = static_cast<float>(ms);
    window_pos = (window_pos + 1) % kWindow;
    if (window_fill < kWindow) {
      ++window_fill;
    }
  }

  double mean_ms() const {return count ? total_ms / static_cast<double>(count) : 0.0;}

  /** @brief Percentile (0..1) over the retained window. */
  double percentile(double p) const
  {
    if (window_fill <= 0) {
      return 0.0;
    }
    std::array<float, kWindow> tmp{};
    std::copy(window.begin(), window.begin() + window_fill, tmp.begin());
    std::sort(tmp.begin(), tmp.begin() + window_fill);
    int idx = static_cast<int>(p * (window_fill - 1) + 0.5);
    idx = std::clamp(idx, 0, window_fill - 1);
    return static_cast<double>(tmp[static_cast<size_t>(idx)]);
  }

  void reset()
  {
    count = 0;
    total_ms = min_ms = max_ms = 0.0;
    window_pos = 0;
    window_fill = 0;
  }
};

class PerfMonitor
{
public:
  bool enable = false;
  /** @brief Emit a report every this many ticks (0 disables periodic output). */
  uint64_t report_every = 0;

  /**
   * @brief Advance the tick counter.
   * @return the formatted report when this tick is a reporting one and periodic
   *         reporting is on, otherwise an empty string. The CALLER logs it: this
   *         class deliberately has no logger, so it stays usable off-ROS (and
   *         inside a unit test) without dragging in a logging framework.
   */
  std::string tick()
  {
    if (!enable) {
      return std::string();
    }
    ++ticks_;
    if (report_every > 0 && (ticks_ % report_every) == 0) {
      return report();
    }
    return std::string();
  }

  void add(Stage s, double ms)
  {
    if (!enable) {
      return;
    }
    const int i = static_cast<int>(s);
    if (i >= 0 && i < static_cast<int>(Stage::kCount)) {
      stats_[static_cast<size_t>(i)].add(ms);
    }
  }

  const StageStats & stats(Stage s) const
  {
    return stats_[static_cast<size_t>(static_cast<int>(s))];
  }

  uint64_t ticks() const {return ticks_;}

  /**
   * @brief Resident set size in kB, read from /proc/self/statm.
   * Returns -1 where unavailable (non-Linux), so a caller can report "n/a"
   * rather than a misleading zero.
   */
  static long residentKb()
  {
#if defined(__linux__)
    std::FILE * f = std::fopen("/proc/self/statm", "r");
    if (!f) {
      return -1;
    }
    long total_pages = 0, resident_pages = 0;
    const int got = std::fscanf(f, "%ld %ld", &total_pages, &resident_pages);
    std::fclose(f);
    if (got != 2) {
      return -1;
    }
    const long page_kb = 4;   // 4 KiB pages on every platform this targets
    return resident_pages * page_kb;
#else
    return -1;
#endif
  }

  /** @brief One human-readable block; call at whatever rate suits the log. */
  std::string report() const
  {
    std::string out;
    char buf[256];
    std::snprintf(buf, sizeof(buf), "--- perf (ticks=%llu, rss=%ld kB) ---\n",
                  static_cast<unsigned long long>(ticks_), residentKb());
    out += buf;
    for (int i = 0; i < static_cast<int>(Stage::kCount); ++i) {
      const auto & st = stats_[static_cast<size_t>(i)];
      if (st.count == 0) {
        continue;
      }
      std::snprintf(buf, sizeof(buf),
                    "  %-11s n=%-8llu mean=%6.3f ms  p95=%6.3f  max=%6.3f  min=%6.3f\n",
                    toString(static_cast<Stage>(i)),
                    static_cast<unsigned long long>(st.count),
                    st.mean_ms(), st.percentile(0.95), st.max_ms, st.min_ms);
      out += buf;
    }
    return out;
  }

  void reset()
  {
    for (auto & s : stats_) {
      s.reset();
    }
    ticks_ = 0;
  }

private:
  std::array<StageStats, static_cast<size_t>(Stage::kCount)> stats_{};
  uint64_t ticks_ = 0;
};

/**
 * @brief RAII timer: measures the enclosing scope into one stage.
 *
 * Declare it as the first statement of the block being measured so the clock
 * read is not counted against the work.
 */
class PerfScope
{
public:
  PerfScope(PerfMonitor & mon, Stage stage)
  : mon_(mon), stage_(stage),
    t0_(mon.enable ? std::chrono::steady_clock::now()
                   : std::chrono::steady_clock::time_point{})
  {}

  ~PerfScope()
  {
    if (!mon_.enable) {
      return;
    }
    const double ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0_).count();
    mon_.add(stage_, ms);
  }

  PerfScope(const PerfScope &) = delete;
  PerfScope & operator=(const PerfScope &) = delete;

private:
  PerfMonitor & mon_;
  Stage stage_;
  std::chrono::steady_clock::time_point t0_;
};

}  // namespace rs_follow

#endif  // RS_FOLLOW_CORE_PERF_MONITOR_HPP
