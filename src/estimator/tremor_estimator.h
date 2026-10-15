// tremor_estimator.h — Tremor / voluntary-motion estimator (Joseph, C5).
//
// C++ port of the MATLAB reference (matlab/):
//   TremorKalman  <- tremor_kalman.m   (task J-2.4)
//   Wflc          <- wflc_freq.m       (task J-4.1)
//   TremorEstimator = Wflc feeding its frequency to TremorKalman
//
// Single precision (the ESP32 FPU is 32-bit), no dynamic allocation, no
// Arduino dependency: the same file is compiled on the ESP32 and on the host
// for the equivalence test (test/host/test_estimator.cpp, compared against
// data/ref/kalman_ref.csv and data/ref/wflc_ref.csv).
//
// Default parameters = matlab/tremor_params.m. Keep the two in sync.

#pragma once

struct EstimatorParams {
  float Ts = 0.002f;  // [s] sample time (500 Hz)

  // Kalman filter (tremor_params.m)
  float q_v = 200.0f;     // voluntary white-acceleration intensity [rad²/s³]
  float q_t = 1000.0f;    // tremor white-acceleration intensity [rad²/s³]
  float rho = 0.9998f;    // oscillator damping per step
  float r_theta = 25.0f * (2.0f * 0.017453292f) * (2.0f * 0.017453292f);  // [rad²]
  float r_omega = 0.05f * 0.05f;                                          // [(rad/s)²]
  float p0 = 1e-2f;       // initial covariance (diagonal)
  bool seed_theta_v = true;  // start theta_v at the first measurement
                             // (false = x0 = 0, as in MATLAB, for the tests)

  // WFLC frequency estimator (tremor_params.m, p.wflc)
  bool adaptive = true;   // false: model frequency fixed at f_fixed
  float f_fixed = 5.0f;   // [Hz]
  float f_lo = 3.0f;      // input band-pass [Hz]
  float f_hi = 10.0f;
  float tau_agc = 1.0f;   // [s] amplitude normalisation time constant
  float gate = 0.25f;     // [rad/s] band RMS below which f is frozen
  float mu0 = 2e-4f;      // frequency adaptation gain
  float mu1 = 5e-3f;      // weight adaptation gain
  float f_init = 5.0f;    // [Hz]
  float f_min = 3.0f;     // [Hz] clamp
  float f_max = 8.0f;
};

// Weighted-frequency Fourier Linear Combiner on the relative gyro rate.
class Wflc {
 public:
  void init(const EstimatorParams& p);
  // One sample of the relative gyro rate [rad/s]; returns f_hat [Hz].
  float step(float y_omega);
  float f_hat() const;
  bool active() const { return active_; }

 private:
  // band-pass biquad (direct form I)
  float b0_ = 0, b2_ = 0, a1_ = 0, a2_ = 0;
  float x1_ = 0, x2_ = 0, u1_ = 0, u2_ = 0;
  // AGC and WFLC state
  float lam_ = 0, gate_ = 0, mu0_ = 0, mu1_ = 0;
  float om_lo_ = 0, om_hi_ = 0, inv_2pi_Ts_ = 0;
  float ms_ = 0, om_ = 0, ph_ = 0, w0_ = 0, w1_ = 0;
  bool active_ = false;
};

// 4-state Kalman filter x = [theta_v, omega_v, theta_t, omega_t].
class TremorKalman {
 public:
  void init(const EstimatorParams& p);
  // One predict + update. f0_hz: model tremor frequency for this sample.
  void step(float y_theta, float y_omega, float f0_hz);
  // Start the voluntary angle at the first measurement (avoids a transient
  // that would otherwise appear in the tremor states at power-up).
  void seed_theta_v(float theta) { x[0] = theta; }

  float x[4] = {};     // state estimate
  float P[4][4] = {};  // covariance

 private:
  void set_frequency(float f0_hz);
  EstimatorParams p_;
  float A_[4][4] = {};
  float Q_[4][4] = {};
  float f_cur_ = -1.0f;  // frequency A_ was built for
};

// Complete estimator: WFLC -> Kalman, one call per sample.
class TremorEstimator {
 public:
  void init(const EstimatorParams& p = EstimatorParams{});
  void step(float y_theta, float y_omega);

  float theta_v() const { return kf_.x[0]; }
  float omega_v() const { return kf_.x[1]; }
  float theta_t() const { return kf_.x[2]; }
  float omega_t() const { return kf_.x[3]; }
  float f_hat() const { return f_; }
  bool adapting() const { return wflc_.active(); }
  const float* state() const { return kf_.x; }

 private:
  EstimatorParams p_;
  Wflc wflc_;
  TremorKalman kf_;
  float f_ = 5.0f;
  bool first_ = true;
};
