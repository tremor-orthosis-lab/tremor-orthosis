// tremor_estimator.cpp — see tremor_estimator.h. Line-by-line port of
// matlab/wflc_freq.m and matlab/tremor_kalman.m, in single precision.

#include "tremor_estimator.h"

#include <math.h>

namespace {
constexpr float TWO_PI = 6.28318530718f;
inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
}  // namespace

// ---------------------------------------------------------------------------
// WFLC
// ---------------------------------------------------------------------------
void Wflc::init(const EstimatorParams& p) {
  // Band-pass coefficients: RBJ biquad, same as bandpass_biquad.m
  const float fc = sqrtf(p.f_lo * p.f_hi);
  const float Qf = fc / (p.f_hi - p.f_lo);
  const float wc = TWO_PI * fc * p.Ts;
  const float al = sinf(wc) / (2.0f * Qf);
  b0_ = al / (1.0f + al);
  b2_ = -b0_;
  a1_ = -2.0f * cosf(wc) / (1.0f + al);
  a2_ = (1.0f - al) / (1.0f + al);
  x1_ = x2_ = u1_ = u2_ = 0.0f;

  lam_ = p.Ts / p.tau_agc;
  gate_ = p.gate;
  mu0_ = p.mu0;
  mu1_ = p.mu1;
  om_lo_ = TWO_PI * p.f_min * p.Ts;
  om_hi_ = TWO_PI * p.f_max * p.Ts;
  inv_2pi_Ts_ = 1.0f / (TWO_PI * p.Ts);

  ms_ = 0.0f;
  om_ = TWO_PI * p.f_init * p.Ts;
  ph_ = 0.0f;
  w0_ = w1_ = 0.0f;
  active_ = false;
}

float Wflc::step(float y_omega) {
  // 1. band-pass (b1 = 0)
  const float uk = b0_ * y_omega + b2_ * x2_ - a1_ * u1_ - a2_ * u2_;
  x2_ = x1_;
  x1_ = y_omega;
  u2_ = u1_;
  u1_ = uk;

  // 2. amplitude normalisation
  ms_ = (1.0f - lam_) * ms_ + lam_ * uk * uk;
  const float rms = sqrtf(ms_);
  const float sk = uk / (rms > gate_ ? rms : gate_);

  // 3. WFLC
  ph_ += om_;
  if (ph_ >= TWO_PI) ph_ -= TWO_PI;
  const float s = sinf(ph_);
  const float c = cosf(ph_);
  const float e = sk - (w0_ * s + w1_ * c);
  active_ = rms > gate_;
  if (active_) {
    om_ = clampf(om_ + 2.0f * mu0_ * e * (w0_ * c - w1_ * s), om_lo_, om_hi_);
  }
  w0_ += 2.0f * mu1_ * e * s;
  w1_ += 2.0f * mu1_ * e * c;
  return om_ * inv_2pi_Ts_;
}

float Wflc::f_hat() const { return om_ * inv_2pi_Ts_; }

// ---------------------------------------------------------------------------
// Kalman filter
// ---------------------------------------------------------------------------
void TremorKalman::init(const EstimatorParams& p) {
  p_ = p;
  const float Ts = p.Ts;
  const float g00 = Ts * Ts * Ts / 3.0f, g01 = Ts * Ts / 2.0f, g11 = Ts;
  for (int i = 0; i < 4; ++i) {
    x[i] = 0.0f;
    for (int j = 0; j < 4; ++j) {
      P[i][j] = (i == j) ? p.p0 : 0.0f;
      Q_[i][j] = 0.0f;
      A_[i][j] = 0.0f;
    }
  }
  Q_[0][0] = p.q_v * g00; Q_[0][1] = Q_[1][0] = p.q_v * g01; Q_[1][1] = p.q_v * g11;
  Q_[2][2] = p.q_t * g00; Q_[2][3] = Q_[3][2] = p.q_t * g01; Q_[3][3] = p.q_t * g11;
  A_[0][0] = 1.0f; A_[0][1] = Ts; A_[1][1] = 1.0f;
  f_cur_ = -1.0f;
}

void TremorKalman::set_frequency(float f0_hz) {
  if (f0_hz == f_cur_) return;  // fixed frequency: built once
  const float w0 = TWO_PI * f0_hz;
  const float c = cosf(w0 * p_.Ts);
  const float s = sinf(w0 * p_.Ts);
  A_[2][2] = p_.rho * c;
  A_[2][3] = p_.rho * s / w0;
  A_[3][2] = -p_.rho * w0 * s;
  A_[3][3] = p_.rho * c;
  f_cur_ = f0_hz;
}

void TremorKalman::step(float y_theta, float y_omega, float f0_hz) {
  set_frequency(f0_hz);
  float (&A)[4][4] = A_;

  // Predict: x = A x
  float xp[4];
  for (int i = 0; i < 4; ++i)
    xp[i] = A[i][0] * x[0] + A[i][1] * x[1] + A[i][2] * x[2] + A[i][3] * x[3];

  // P = A P A' + Q
  float AP[4][4];
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      AP[i][j] = A[i][0] * P[0][j] + A[i][1] * P[1][j] + A[i][2] * P[2][j] + A[i][3] * P[3][j];
  float Pp[4][4];
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      Pp[i][j] = AP[i][0] * A[j][0] + AP[i][1] * A[j][1] + AP[i][2] * A[j][2] +
                 AP[i][3] * A[j][3] + Q_[i][j];

  // H = [1 0 1 0; 0 1 0 1]  ->  P H' has columns (P[:,0]+P[:,2], P[:,1]+P[:,3])
  float PHt[4][2];
  for (int i = 0; i < 4; ++i) {
    PHt[i][0] = Pp[i][0] + Pp[i][2];
    PHt[i][1] = Pp[i][1] + Pp[i][3];
  }
  // S = H P H' + R  (2x2)
  const float S00 = PHt[0][0] + PHt[2][0] + p_.r_theta;
  const float S01 = PHt[0][1] + PHt[2][1];
  const float S10 = PHt[1][0] + PHt[3][0];
  const float S11 = PHt[1][1] + PHt[3][1] + p_.r_omega;
  const float det = S00 * S11 - S01 * S10;
  const float i00 = S11 / det, i01 = -S01 / det, i10 = -S10 / det, i11 = S00 / det;

  // K = P H' S^-1  (4x2)
  float K[4][2];
  for (int i = 0; i < 4; ++i) {
    K[i][0] = PHt[i][0] * i00 + PHt[i][1] * i10;
    K[i][1] = PHt[i][0] * i01 + PHt[i][1] * i11;
  }

  // Innovation and state update
  const float r0 = y_theta - (xp[0] + xp[2]);
  const float r1 = y_omega - (xp[1] + xp[3]);
  for (int i = 0; i < 4; ++i) x[i] = xp[i] + K[i][0] * r0 + K[i][1] * r1;

  // P = (I - K H) P, then symmetrise.  (K H)[i][j] = K[i][j%2] for H above
  float Pn[4][4];
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j) {
      float kh_p = 0.0f;
      for (int m = 0; m < 4; ++m) kh_p += K[i][m % 2] * Pp[m][j];
      Pn[i][j] = Pp[i][j] - kh_p;
    }
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j) P[i][j] = 0.5f * (Pn[i][j] + Pn[j][i]);
}

// ---------------------------------------------------------------------------
// WFLC -> Kalman
// ---------------------------------------------------------------------------
void TremorEstimator::init(const EstimatorParams& p) {
  p_ = p;
  wflc_.init(p);
  kf_.init(p);
  f_ = p.adaptive ? p.f_init : p.f_fixed;
  first_ = true;
}

void TremorEstimator::step(float y_theta, float y_omega) {
  if (first_) {
    if (p_.seed_theta_v) kf_.seed_theta_v(y_theta);
    first_ = false;
  }
  f_ = p_.adaptive ? wflc_.step(y_omega) : p_.f_fixed;
  kf_.step(y_theta, y_omega, f_);
}
