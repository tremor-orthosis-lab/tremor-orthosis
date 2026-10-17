// openloop_test.cpp — Stand-in for the controller (C5), used to test the
// mechanics without the estimator: returns tau = A · sin(2·pi·f·t).
// Built when USE_OPENLOOP_TEST is defined (env "openloop"). Tasks L-2.3, L-2.5.
// Amplitude and frequency can be changed at run time with
//   SET ol_amp <N·m>   and   SET ol_freq <Hz>

#ifdef USE_OPENLOOP_TEST

#include <math.h>

#include "contracts.h"

float g_openloop_amp_Nm = 0.05f;
float g_openloop_freq_Hz = 2.0f;

namespace {
DebugState dbg;
float t_s = 0.0f;
}  // namespace

void controller_init(const Gains& /*g*/) {
  dbg = DebugState{};
  t_s = 0.0f;
}

float controller_step(const ImuSample& /*s1*/, const ImuSample& /*s2*/, float dt) {
  t_s += dt;
  dbg.tau_cmd = g_openloop_amp_Nm * sinf(2.0f * (float)M_PI * g_openloop_freq_Hz * t_s);
  return dbg.tau_cmd;
}

void controller_get_debug(DebugState* d) { *d = dbg; }

// Modes of the real controller: not available in this stand-in.
void controller_set_enabled(bool /*on*/) {}
bool controller_enabled() { return false; }
bool controller_start_ident(float, float, float, float) { return false; }
void controller_stop_ident() {}
bool controller_ident_running() { return false; }

#endif  // USE_OPENLOOP_TEST
