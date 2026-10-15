// controller.cpp — controller_step() (C5), Joseph's side.
//
// v1 (J-2.4 + J-4.1): IMU pair -> wrist measurement -> tremor estimator
// (WFLC + Kalman, src/estimator/). The torque is still ZERO: the LQR
// (J-3.1, tau = -K * x_hat) is not designed yet. With this version the
// "mock" firmware logs theta_v_hat, theta_t_hat and f_hat in real time.
//
// Not built when USE_OPENLOOP_TEST is defined (Louis's open-loop stub).

#ifndef USE_OPENLOOP_TEST

#include <math.h>

#include "contracts.h"
#include "../estimator/tremor_estimator.h"

namespace {
Gains gains;
DebugState dbg;
TremorEstimator estimator;

constexpr float PI_F = 3.14159265f;

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
// TODO(J-1.3): check both signs with the real kit (bend the hand in
// extension: theta_hat and the gyro difference must both increase).
void wrist_measurement(const ImuSample& s1, const ImuSample& s2, float* theta, float* omega) {
  const float a1 = atan2f(-s1.acc[0], s1.acc[2]);
  const float a2 = atan2f(-s2.acc[0], s2.acc[2]);
  float th = a1 - a2;
  if (th > PI_F) th -= 2.0f * PI_F;
  if (th < -PI_F) th += 2.0f * PI_F;
  *theta = th;
  *omega = s1.gyr[1] - s2.gyr[1];
}
}  // namespace

void controller_init(const Gains& g) {
  gains = g;
  dbg = DebugState{};
  // Estimator tuning lives in EstimatorParams (= matlab/tremor_params.m).
  estimator.init(EstimatorParams{});
}

float controller_step(const ImuSample& s1, const ImuSample& s2, float /*dt*/) {
  float theta_mes, omega_mes;
  wrist_measurement(s1, s2, &theta_mes, &omega_mes);
  estimator.step(theta_mes, omega_mes);

  dbg.theta_v_hat = estimator.theta_v();
  dbg.theta_t_hat = estimator.theta_t();
  dbg.theta_hat = dbg.theta_v_hat + dbg.theta_t_hat;
  dbg.f_hat = estimator.f_hat();

  dbg.tau_cmd = 0.0f;  // J-3.1: tau = -K(f_hat) * x_hat
  return dbg.tau_cmd;
}

void controller_get_debug(DebugState* d) { *d = dbg; }

#endif  // USE_OPENLOOP_TEST
