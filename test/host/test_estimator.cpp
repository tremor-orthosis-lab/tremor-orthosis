// test_estimator.cpp — Host-side equivalence test: C++ estimator vs MATLAB.
//
// Runs src/estimator/tremor_estimator.cpp (single precision) on the inputs
// stored in data/ref/*.csv and compares its outputs with the MATLAB outputs
// (double precision) stored in the same files.
//
//   kalman_ref.csv : Kalman filter alone, model frequency fixed at 5 Hz
//                    (matlab/run_kalman_synthetic.m)
//   wflc_ref.csv   : WFLC + Kalman, tremor chirp 4 -> 6 Hz
//                    (matlab/run_adaptive_synthetic.m)
//
// Build and run from the repository root (any C++17 compiler):
//   g++ -std=c++17 -O2 -Isrc/estimator test/host/test_estimator.cpp src/estimator/tremor_estimator.cpp -o test_estimator && ./test_estimator
//
// Exit code 0 = all checks pass. Re-run after any change to the estimator or
// to matlab/tremor_params.m (and regenerate the CSVs from MATLAB).

#include <math.h>
#include <stdio.h>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "tremor_estimator.h"

namespace {

std::vector<std::vector<double>> read_csv(const std::string& path) {
  std::vector<std::vector<double>> rows;
  std::ifstream f(path);
  if (!f) {
    fprintf(stderr, "cannot open %s (run from the repository root)\n", path.c_str());
    return rows;
  }
  std::string line;
  std::getline(f, line);  // header
  while (std::getline(f, line)) {
    std::vector<double> r;
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) r.push_back(std::stod(cell));
    if (!r.empty()) rows.push_back(r);
  }
  return rows;
}

int failures = 0;

void check(const char* what, double value, double limit, const char* unit) {
  const bool ok = value <= limit;
  printf("  %-44s %10.3g %-6s (limit %.3g)  %s\n", what, value, unit, limit, ok ? "OK" : "FAIL");
  if (!ok) ++failures;
}

constexpr double RAD2DEG = 57.29577951308232;

}  // namespace

int main(int argc, char** argv) {
  const std::string dir = argc > 1 ? argv[1] : "data/ref";

  // ---- 1. Kalman filter, f0 = 5 Hz --------------------------------------
  {
    const auto d = read_csv(dir + "/kalman_ref.csv");
    if (d.empty()) return 2;
    EstimatorParams p;
    p.seed_theta_v = false;  // MATLAB starts from x0 = 0
    TremorKalman kf;
    kf.init(p);
    double err[4] = {0, 0, 0, 0}, ref_rms_t = 0;
    for (const auto& r : d) {
      // columns: t, y_theta, y_omega, theta_v, omega_v, theta_t, omega_t
      kf.step((float)r[1], (float)r[2], 5.0f);
      for (int i = 0; i < 4; ++i) err[i] = fmax(err[i], fabs(kf.x[i] - r[3 + i]));
      ref_rms_t += r[5] * r[5];
    }
    ref_rms_t = sqrt(ref_rms_t / d.size());
    printf("Kalman filter, f0 = 5 Hz, %zu samples (tremor RMS %.2f deg)\n", d.size(),
           ref_rms_t * RAD2DEG);
    check("max |theta_v - MATLAB|", err[0] * RAD2DEG, 0.01, "deg");
    check("max |theta_t - MATLAB|", err[2] * RAD2DEG, 0.01, "deg");
    check("max |omega_v - MATLAB|", err[1], 0.005, "rad/s");
    check("max |omega_t - MATLAB|", err[3], 0.005, "rad/s");
  }

  // ---- 2. WFLC + Kalman, chirp 4 -> 6 Hz ----------------------------------
  {
    const auto d = read_csv(dir + "/wflc_ref.csv");
    if (d.empty()) return 2;
    EstimatorParams p;
    p.seed_theta_v = false;
    TremorEstimator est;
    est.init(p);
    double f_err = 0, t_err2 = 0, ref2 = 0;
    long gate_mismatch = 0;
    for (const auto& r : d) {
      // columns: t, y_theta, y_omega, f_hat, active, theta_t_hat
      est.step((float)r[1], (float)r[2]);
      f_err = fmax(f_err, fabs(est.f_hat() - r[3]));
      if ((int)est.adapting() != (int)r[4]) ++gate_mismatch;
      t_err2 += (est.theta_t() - r[5]) * (est.theta_t() - r[5]);
      ref2 += r[5] * r[5];
    }
    printf("WFLC + Kalman, chirp 4 -> 6 Hz, %zu samples\n", d.size());
    check("max |f_hat - MATLAB|", f_err, 0.02, "Hz");
    check("RMS |theta_t - MATLAB| / RMS theta_t", 100.0 * sqrt(t_err2 / ref2), 1.0, "%");
    check("samples where the gate differs", (double)gate_mismatch, 20, "");
  }

  printf(failures ? "\n%d check(s) FAILED\n" : "\nAll checks passed\n", failures);
  return failures ? 1 : 0;
}
