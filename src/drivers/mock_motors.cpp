// mock_motors.cpp — Stand-in for the real motor drivers (C5).
//
// Built when USE_MOCK_MOTORS is defined (default env "mock"). It drives no
// hardware: it only stores the saturated torque so that it appears in the log.
// This lets the controller be developed and tested on a kit with IMUs only.
// The real implementation (drivers/motors.cpp, SimpleFOC) is task L-2.3.

#ifdef USE_MOCK_MOTORS

#include <Arduino.h>

#include "contracts.h"

namespace {
bool enabled = false;
float last_tau = 0.0f;
}  // namespace

void motors_init() {
  enabled = false;
  last_tau = 0.0f;
}

void motors_set_torque(float tau_wrist_Nm) {
  if (!enabled) {
    last_tau = 0.0f;
    return;
  }
  last_tau = constrain(tau_wrist_Nm, -safety::TAU_MAX_NM, safety::TAU_MAX_NM);
}

void motors_disable() {
  enabled = false;
  last_tau = 0.0f;
}

bool motors_enabled() { return enabled; }

// Used by the ARM command; the real driver also raises the EN pins.
void motors_enable() { enabled = true; }

float motors_get_angle(int /*id*/) { return 0.0f; }

#endif  // USE_MOCK_MOTORS
