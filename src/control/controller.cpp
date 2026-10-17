// controller.cpp — controller_step() (C5), Joseph's side.
//
// Every sample (2 ms):
//   1. wrist angle and rate from the two IMUs (signs: C2)
//   2. tremor estimator: WFLC + Kalman (src/estimator/)
//   3. torque, depending on the mode:
//      - OFF (default)  : tau = 0, the estimator still runs and is logged
//      - CTRL on  (J-3.1): tau = -K(f_hat)·x_hat, K interpolated in f_hat
//                          (default gains: virtual damper on omega_t,
//                          matlab/design_controller.m), ramped in over 1 s
//      - IDENT    (J-3.3): chirp torque for system identification
//                          (matlab/identify.m), replaces the controller
// The torque is only applied if the motors are ARMed, and main.cpp clamps
// it to tau_max. Not built when USE_OPENLOOP_TEST is defined.

#ifndef USE_OPENLOOP_TEST

#include <math.h>

#include "../estimator/tremor_estimator.h"
#include "contracts.h"

namespace {
Gains gains;
DebugState dbg;
TremorEstimator estimator;

constexpr float PI_F = 3.14159265f;

bool enabled = false;
float ramp = 0.0f;  // 0 -> 1 over RAMP_S when the controller is switched on
constexpr float RAMP_S = 1.0f;

struct Ident {
  bool running = false;
  float amp = 0, f1 = 0, f2 = 0, dur = 0, t = 0, phase = 0;
} ident;
constexpr float IDENT_FADE_S = 0.5f;

// Wrist flexion/extension from the two IMUs (frames and signs: C2).
//
// Each IMU's rotation about its y axis, from gravity: alpha = atan2(-ax, az)
// (valid when the accelerations of the hand are small compared with g, and
// when the wrist axis y is not vertical). A positive rotation about y turns
// x (towards the hand) towards -z (palmar), i.e. FLEXION, so
//   theta = alpha1 - alpha2          (extension positive)
//   omega = gyr1_y - gyr2_y
// The accelerometer angle is only weakly trusted by the Kalman filter
// (r_theta), the gyro carries the tremor.
// Checked with the WATCH command at first power-up (docs/bringup.md).
void wrist_measurement(const ImuSample& s1, const ImuSample& s2, float* theta, float* omega) {
  const float a1 = atan2f(-s1.acc[0], s1.acc[2]);
  const float a2 = atan2f(-s2.acc[0], s2.acc[2]);
  float th = a1 - a2;
  if (th > PI_F) th -= 2.0f * PI_F;
  if (th < -PI_F) th += 2.0f * PI_F;
  *theta = th;
  *omega = s1.gyr[1] - s2.gyr[1];
}

// Gain row for frequency f: linear interpolation in the table, clamped at
// the ends.
void gain_row(float f, float k[N_STATE]) {
  int i = 0;
  if (f <= gains.f0_hz[0]) {
    for (int j = 0; j < N_STATE; ++j) k[j] = gains.K[0][j];
    return;
  }
  if (f >= gains.f0_hz[N_F0 - 1]) {
    for (int j = 0; j < N_STATE; ++j) k[j] = gains.K[N_F0 - 1][j];
    return;
  }
  while (i < N_F0 - 2 && f > gains.f0_hz[i + 1]) ++i;
  const float w = (f - gains.f0_hz[i]) / (gains.f0_hz[i + 1] - gains.f0_hz[i]);
  for (int j = 0; j < N_STATE; ++j) k[j] = (1.0f - w) * gains.K[i][j] + w * gains.K[i + 1][j];
}

float ident_step(float dt) {
  Ident& id = ident;
  const float f = id.f1 + (id.f2 - id.f1) * id.t / id.dur;  // linear chirp
  id.phase += 2.0f * PI_F * f * dt;
  if (id.phase > 2.0f * PI_F) id.phase -= 2.0f * PI_F;
  float env = 1.0f;  // fade in / out to avoid a torque step
  if (id.t < IDENT_FADE_S) env = id.t / IDENT_FADE_S;
  if (id.dur - id.t < IDENT_FADE_S) env = (id.dur - id.t) / IDENT_FADE_S;
  const float tau = id.amp * env * sinf(id.phase);
  id.t += dt;
  if (id.t >= id.dur) id.running = false;
  return tau;
}
}  // namespace

void controller_init(const Gains& g) {
  gains = g;
  dbg = DebugState{};
  // Estimator tuning lives in EstimatorParams (= matlab/tremor_params.m).
  estimator.init(EstimatorParams{});
  enabled = false;
  ramp = 0.0f;
  ident = Ident{};
}

float controller_step(const ImuSample& s1, const ImuSample& s2, float dt) {
  float theta_mes, omega_mes;
  wrist_measurement(s1, s2, &theta_mes, &omega_mes);
  estimator.step(theta_mes, omega_mes);

  dbg.theta_v_hat = estimator.theta_v();
  dbg.theta_t_hat = estimator.theta_t();
  dbg.theta_hat = dbg.theta_v_hat + dbg.theta_t_hat;
  dbg.f_hat = estimator.f_hat();

  float tau = 0.0f;
  if (ident.running) {
    tau = ident_step(dt);
  } else if (enabled) {
    ramp = fminf(1.0f, ramp + dt / RAMP_S);
    float k[N_STATE];
    gain_row(dbg.f_hat, k);
    const float* x = estimator.state();
    tau = -ramp * (k[0] * x[0] + k[1] * x[1] + k[2] * x[2] + k[3] * x[3]);
  } else {
    ramp = 0.0f;
  }
  dbg.tau_cmd = tau;
  return tau;
}

void controller_get_debug(DebugState* d) { *d = dbg; }

void controller_set_enabled(bool on) {
  enabled = on;
  if (!on) ramp = 0.0f;
}

bool controller_enabled() { return enabled; }

bool controller_start_ident(float amp_Nm, float f1_Hz, float f2_Hz, float dur_s) {
  if (amp_Nm <= 0.0f || amp_Nm > safety::TAU_MAX_NM || f1_Hz <= 0.0f || f2_Hz <= 0.0f ||
      f2_Hz > 50.0f || dur_s < 2.0f * IDENT_FADE_S) {
    return false;
  }
  ident = Ident{true, amp_Nm, f1_Hz, f2_Hz, dur_s, 0.0f, 0.0f};
  return true;
}

void controller_stop_ident() { ident.running = false; }

bool controller_ident_running() { return ident.running; }

#endif  // USE_OPENLOOP_TEST
