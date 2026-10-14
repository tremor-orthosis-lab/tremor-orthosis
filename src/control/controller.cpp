// controller.cpp — controller_step() (C5), Joseph's side.
//
// v0 (J-1.1): placeholder that returns zero torque, so the whole loop runs
// end to end. To be replaced by:
//   J-2.4  Kalman filter (src/estimator/kalman.cpp)
//   J-3.1  discrete LQR, tau = -K · x_hat
//   J-4.1  adaptive tremor-frequency estimator
//
// Not built when USE_OPENLOOP_TEST is defined (Louis's open-loop stub).

#ifndef USE_OPENLOOP_TEST

#include "contracts.h"

namespace {
Gains gains;
DebugState dbg;
}  // namespace

void controller_init(const Gains& g) {
  gains = g;
  dbg = DebugState{};
}

float controller_step(const ImuSample& /*s1*/, const ImuSample& /*s2*/, float /*dt*/) {
  dbg.tau_cmd = 0.0f;
  return dbg.tau_cmd;
}

void controller_get_debug(DebugState* d) { *d = dbg; }

#endif  // USE_OPENLOOP_TEST
