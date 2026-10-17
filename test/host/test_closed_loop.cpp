// test_closed_loop.cpp — Host-side closed-loop test of the REAL firmware
// controller (src/control/controller.cpp + src/estimator/) on a simulated
// wrist (task J-3.1).
//
// The wrist is the model of matlab/plant_model.m (contracts C4):
//   J th'' + B th' + K th = tau_tremor + tau_voluntary + tau_motor
// with a 6 ms delay on the motor torque. The two IMUs are simulated with the
// frames and signs of C2 (forearm IMU still, hand IMU at alpha1 - theta,
// gyro y of the hand = -dtheta/dt, accelerometers = gravity + hand
// acceleration), so this test also checks the sign chain
//   IMU -> wrist_measurement -> estimator -> controller -> torque.
// A sign error anywhere makes the controller amplify the tremor: FAIL.
//
// Build and run from the repository root:
//   g++ -std=c++17 -O2 -Iinclude -Isrc/estimator test/host/test_closed_loop.cpp src/control/controller.cpp src/estimator/tremor_estimator.cpp -o test_closed_loop && ./test_closed_loop
//
// Expected (matlab/sim_closed_loop.m): ~55-62 % attenuation at 4-6 Hz,
// voluntary motion at 0.5 Hz changed by less than 10 %.

#include <math.h>
#include <stdio.h>

#include <vector>

#include "contracts.h"

namespace {

constexpr double PI = 3.14159265358979;
constexpr double G = 9.80665;
constexpr double TS = 0.002;
constexpr int DELAY = 3;  // samples (6 ms)

struct Result {
  std::vector<double> theta;
  double tau_peak = 0;
};

// Simulates T seconds. Tremor: free amplitude amp_deg at f_trem (0 = none).
// Voluntary: free 20 deg at 0.5 Hz if vol. Controller on if ctrl.
Result simulate(double f_trem, double amp_deg, bool vol, bool ctrl, double T) {
  const double J = mech::J_HAND, K = mech::K_WRIST, B = mech::B_WRIST;
  controller_init(Gains{});
  controller_set_enabled(ctrl);

  const int N = (int)(T / TS);
  const int sub = 20;
  const double h = TS / sub;
  double th = 0, om = 0;
  std::vector<double> buf(DELAY, 0.0);
  unsigned seed = 12345;
  auto randn = [&seed]() {  // small deterministic noise
    double s = 0;
    for (int i = 0; i < 12; ++i) {
      seed = seed * 1103515245u + 12345u;
      s += (double)((seed >> 8) & 0xFFFF) / 65536.0;
    }
    return s - 6.0;
  };

  // Torques that give the chosen free motion (inverse dynamics of sines)
  auto tau_ext = [&](double t) {
    double tau = 0;
    if (amp_deg > 0) {
      const double w = 2 * PI * f_trem, a = amp_deg * PI / 180;
      tau += a * ((K - J * w * w) * sin(w * t) + B * w * cos(w * t));
    }
    if (vol) {
      const double w = 2 * PI * 0.5, a = 20 * PI / 180;
      tau += a * ((K - J * w * w) * sin(w * t) + B * w * cos(w * t));
    }
    return tau;
  };

  Result r;
  r.theta.resize(N);
  const double a1 = 3 * PI / 180, rx = 0.07, rz = 0.02;
  double om_prev = 0;
  for (int k = 0; k < N; ++k) {
    const double t = k * TS;
    // --- sensors (C2 frames)
    const double a2 = a1 - th;
    const double w2 = -om, dw2 = -(om - om_prev) / TS;
    ImuSample s1{}, s2{};
    s1.acc[0] = (float)(-G * sin(a1) + 0.04 * randn());
    s1.acc[2] = (float)(G * cos(a1) + 0.04 * randn());
    s1.gyr[1] = (float)(0.002 * randn());
    s2.acc[0] = (float)(-G * sin(a2) + dw2 * rz - w2 * w2 * rx + 0.04 * randn());
    s2.acc[2] = (float)(G * cos(a2) - dw2 * rx - w2 * w2 * rz + 0.04 * randn());
    s2.gyr[1] = (float)(w2 + 0.002 * randn());
    om_prev = om;

    // --- firmware (same clamp as main.cpp)
    float tau = controller_step(s1, s2, (float)TS);
    tau = fmaxf(-safety::TAU_MAX_NM, fminf(safety::TAU_MAX_NM, tau));
    r.tau_peak = fmax(r.tau_peak, fabs(tau));
    buf.insert(buf.begin(), tau);
    const double u = buf.back();
    buf.pop_back();

    // --- wrist (sub-stepped semi-implicit Euler)
    r.theta[k] = th;
    for (int i = 0; i < sub; ++i) {
      const double acc = (u + tau_ext(t + i * h) - B * om - K * th) / J;
      om += h * acc;
      th += h * om;
    }
  }
  return r;
}

double rms_after(const std::vector<double>& x, double t0) {
  double s = 0, m = 0;
  int n = 0;
  for (size_t k = (size_t)(t0 / TS); k < x.size(); ++k, ++n) m += x[k];
  m /= n;
  for (size_t k = (size_t)(t0 / TS); k < x.size(); ++k) s += (x[k] - m) * (x[k] - m);
  return sqrt(s / n);
}

}  // namespace

int main() {
  int fail = 0;
  printf("Closed loop, firmware controller on the simulated wrist\n");
  for (double f : {4.0, 5.0, 6.0}) {
    const Result off = simulate(f, 5, false, false, 20);
    const Result on = simulate(f, 5, false, true, 20);
    const double att = 1 - rms_after(on.theta, 5) / rms_after(off.theta, 5);
    const bool ok = att > 0.50;  // plan criterion (J-2.3)
    printf("  tremor %.0f Hz, 5 deg: attenuation %3.0f %%, peak torque %.3f N m  %s\n", f,
           100 * att, on.tau_peak, ok ? "OK" : "FAIL");
    if (!ok) ++fail;
  }
  const Result voff = simulate(0, 0, true, false, 20);
  const Result von = simulate(0, 0, true, true, 20);
  const double ch = rms_after(von.theta, 5) / rms_after(voff.theta, 5) - 1;
  const bool vok = fabs(ch) < 0.10;
  printf("  voluntary 0.5 Hz, 20 deg, no tremor: amplitude change %+.1f %%  %s\n", 100 * ch,
         vok ? "OK" : "FAIL");
  if (!vok) ++fail;

  printf(fail ? "\n%d check(s) FAILED\n" : "\nAll checks passed\n", fail);
  return fail ? 1 : 0;
}
