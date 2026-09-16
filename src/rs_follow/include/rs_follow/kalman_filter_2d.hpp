/**
 * @file kalman_filter_2d.hpp
 * @brief 2D constant-velocity Kalman filter for target position smoothing.
 *
 * Adapted from jie_deamon/include/kalman_filter.hpp (MIT License),
 * https://github.com/6-robot/jie_deamon
 */

#ifndef RS_FOLLOW_KALMAN_FILTER_2D_HPP
#define RS_FOLLOW_KALMAN_FILTER_2D_HPP

#include <array>
#include <cmath>
#include <chrono>

namespace rs_follow
{

class KalmanFilter2D
{
public:
  KalmanFilter2D(double process_noise = 0.1, double measurement_noise = 0.05)
  : q_(process_noise), r_(measurement_noise)
  {
    reset();
  }

  void setProcessNoise(double q) { q_ = q; }
  void setMeasurementNoise(double r) { r_ = r; }
  // Reject measurements whose Mahalanobis distance exceeds `gate` sigmas (0 = off).
  void setGate(double gate) { gate_ = gate; }
  // Scale measurement noise as r * max(1, n_ref / n_points); 0 disables scaling.
  void setMeasurementNoiseScale(int n_ref) { n_ref_ = n_ref; }

  void reset()
  {
    initialized_ = false;
    x_ = {0.0, 0.0, 0.0, 0.0};
    P_.fill(0.0);
    P_[0] = 1.0;
    P_[5] = 1.0;
    P_[10] = 10.0;
    P_[15] = 10.0;
  }

  void update(double meas_x, double meas_y, double & fx, double & fy,
              bool * accepted = nullptr, int n_points = 0)
  {
    auto now = std::chrono::steady_clock::now();
    if (!initialized_) {
      x_[0] = meas_x;
      x_[1] = meas_y;
      x_[2] = 0.0;
      x_[3] = 0.0;
      last_time_ = now;
      initialized_ = true;
      fx = meas_x;
      fy = meas_y;
      if (accepted) {*accepted = true;}
      return;
    }

    double dt = std::chrono::duration<double>(now - last_time_).count();
    last_time_ = now;
    if (dt <= 0.0 || dt > 1.0) {
      dt = 0.1;
    }

    predict(dt);
    const bool acc = correctGated(meas_x, meas_y, n_points);
    if (accepted) {*accepted = acc;}
    fx = x_[0];
    fy = x_[1];
  }

  void predictOnly(double dt, double & px, double & py)
  {
    if (!initialized_) {
      px = 0.0;
      py = 0.0;
      return;
    }
    predict(dt);
    px = x_[0];
    py = x_[1];
  }

  void setState(double x, double y)
  {
    x_[0] = x;
    x_[1] = y;
    x_[2] = 0.0;
    x_[3] = 0.0;
    P_.fill(0.0);
    P_[0] = 0.5;
    P_[5] = 0.5;
    P_[10] = 10.0;
    P_[15] = 10.0;
    initialized_ = true;
    last_time_ = std::chrono::steady_clock::now();
  }

  bool isInitialized() const { return initialized_; }
  double getVelocityX() const { return x_[2]; }
  double getVelocityY() const { return x_[3]; }

private:
  void predict(double dt)
  {
    x_[0] += x_[2] * dt;
    x_[1] += x_[3] * dt;

    double dt2 = dt * dt;
    double dt3 = dt2 * dt;

    std::array<double, 16> FP;
    FP[0] = P_[0] + dt * P_[8];
    FP[1] = P_[1] + dt * P_[9];
    FP[2] = P_[2] + dt * P_[10];
    FP[3] = P_[3] + dt * P_[11];
    FP[4] = P_[4] + dt * P_[12];
    FP[5] = P_[5] + dt * P_[13];
    FP[6] = P_[6] + dt * P_[14];
    FP[7] = P_[7] + dt * P_[15];
    FP[8] = P_[8];
    FP[9] = P_[9];
    FP[10] = P_[10];
    FP[11] = P_[11];
    FP[12] = P_[12];
    FP[13] = P_[13];
    FP[14] = P_[14];
    FP[15] = P_[15];

    P_[0] = FP[0] + FP[2] * dt + q_ * dt3 / 3.0;
    P_[1] = FP[1] + FP[3] * dt;
    P_[2] = FP[2] + q_ * dt2 / 2.0;
    P_[3] = FP[3];

    P_[4] = FP[4] + FP[6] * dt;
    P_[5] = FP[5] + FP[7] * dt + q_ * dt3 / 3.0;
    P_[6] = FP[6] + q_ * dt2 / 2.0;
    P_[7] = FP[7];

    P_[8] = FP[8] + FP[10] * dt + q_ * dt2 / 2.0;
    P_[9] = FP[9] + FP[11] * dt;
    P_[10] = FP[10] + q_ * dt;
    P_[11] = FP[11];

    P_[12] = FP[12] + FP[14] * dt;
    P_[13] = FP[13] + FP[15] * dt + q_ * dt2 / 2.0;
    P_[14] = FP[14] + q_ * dt;
    P_[15] = FP[15];
  }

  bool correctGated(double meas_x, double meas_y, int n_points)
  {
    double y0 = meas_x - x_[0];
    double y1 = meas_y - x_[1];

    // adaptive measurement noise: fewer points -> distrust the measurement
    double r_eff = r_;
    if (n_ref_ > 0 && n_points > 0) {
      r_eff *= std::max(1.0, static_cast<double>(n_ref_) / n_points);
    }

    double s00 = P_[0] + r_eff;
    double s01 = P_[1];
    double s10 = P_[4];
    double s11 = P_[5] + r_eff;

    double det = s00 * s11 - s01 * s10;
    if (std::abs(det) < 1e-12) {
      return false;
    }
    double inv_det = 1.0 / det;
    double si00 = s11 * inv_det;
    double si01 = -s01 * inv_det;
    double si10 = -s10 * inv_det;
    double si11 = s00 * inv_det;

    // Mahalanobis gating: drop gross outliers instead of jumping the estimate
    if (gate_ > 0.0) {
      const double d2 = y0 * (si00 * y0 + si01 * y1) + y1 * (si10 * y0 + si11 * y1);
      if (d2 > gate_ * gate_) {
        return false;
      }
    }

    double k00 = P_[0] * si00 + P_[1] * si10;
    double k01 = P_[0] * si01 + P_[1] * si11;
    double k10 = P_[4] * si00 + P_[5] * si10;
    double k11 = P_[4] * si01 + P_[5] * si11;
    double k20 = P_[8] * si00 + P_[9] * si10;
    double k21 = P_[8] * si01 + P_[9] * si11;
    double k30 = P_[12] * si00 + P_[13] * si10;
    double k31 = P_[12] * si01 + P_[13] * si11;

    x_[0] += k00 * y0 + k01 * y1;
    x_[1] += k10 * y0 + k11 * y1;
    x_[2] += k20 * y0 + k21 * y1;
    x_[3] += k30 * y0 + k31 * y1;

    auto P_old = P_;
    std::array<double, 16> P_new;
    P_new[0] = (1.0 - k00) * P_old[0] - k01 * P_old[4];
    P_new[1] = (1.0 - k00) * P_old[1] - k01 * P_old[5];
    P_new[2] = (1.0 - k00) * P_old[2] - k01 * P_old[6];
    P_new[3] = (1.0 - k00) * P_old[3] - k01 * P_old[7];
    P_new[4] = -k10 * P_old[0] + (1.0 - k11) * P_old[4];
    P_new[5] = -k10 * P_old[1] + (1.0 - k11) * P_old[5];
    P_new[6] = -k10 * P_old[2] + (1.0 - k11) * P_old[6];
    P_new[7] = -k10 * P_old[3] + (1.0 - k11) * P_old[7];
    P_new[8] = -k20 * P_old[0] - k21 * P_old[4] + P_old[8];
    P_new[9] = -k20 * P_old[1] - k21 * P_old[5] + P_old[9];
    P_new[10] = -k20 * P_old[2] - k21 * P_old[6] + P_old[10];
    P_new[11] = -k20 * P_old[3] - k21 * P_old[7] + P_old[11];
    P_new[12] = -k30 * P_old[0] - k31 * P_old[4] + P_old[12];
    P_new[13] = -k30 * P_old[1] - k31 * P_old[5] + P_old[13];
    P_new[14] = -k30 * P_old[2] - k31 * P_old[6] + P_old[14];
    P_new[15] = -k30 * P_old[3] - k31 * P_old[7] + P_old[15];
    P_ = P_new;
    return true;
  }

  double q_;
  double r_;
  double gate_ = 0.0;
  int n_ref_ = 0;
  bool initialized_ = false;
  std::array<double, 4> x_;
  std::array<double, 16> P_;
  std::chrono::steady_clock::time_point last_time_;
};

}  // namespace rs_follow

#endif  // RS_FOLLOW_KALMAN_FILTER_2D_HPP
