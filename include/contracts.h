// contracts.h — Interface contracts shared by every part of the firmware.
//
// This file is the single source of truth for the contracts C1–C7 agreed at
// kick-off (S0). Any change here must be announced to the other teammate
// before it is merged into main.
//
// Ownership (C5):
//   src/drivers/    -> Louis  (motors, IMU, power, safety)
//   src/estimator/  -> Joseph (Kalman filter, frequency estimator)
//   src/control/    -> Joseph (LQR, controller_step)
//   src/main.cpp    -> shared (500 Hz loop, serial commands, logging)

#pragma once
#include <stdint.h>

// ---------------------------------------------------------------------------
// C1 · ESP32 pinout (identical on all three boards: Joseph's kit, orthosis,
//      test bench)
// ---------------------------------------------------------------------------
namespace pins {
constexpr int I2C_SDA = 21;
constexpr int I2C_SCL = 22;
constexpr uint32_t I2C_FREQ_HZ = 400000;

constexpr uint8_t IMU1_ADDR = 0x68;  // forearm, AD0 -> GND
constexpr uint8_t IMU2_ADDR = 0x69;  // hand shell, AD0 -> 3.3 V

// Motor A pulls the palmar cable (flexion, tau < 0)
constexpr int MOTOR_A_PWM[3] = {25, 26, 27};
constexpr int MOTOR_A_EN = 14;
// Motor B pulls the dorsal cable (extension, tau > 0)
constexpr int MOTOR_B_PWM[3] = {32, 33, 4};
constexpr int MOTOR_B_EN = 17;

constexpr int ENC_A_ANALOG = 34;  // AS5600 analog output, motor A
constexpr int ENC_B_ANALOG = 35;  // AS5600 analog output, motor B

constexpr int VBAT_ADC = 36;      // divider 100 k / 20 k
constexpr int ESTOP = 23;         // button to GND, internal pull-up
constexpr int BENCH_SYNC = 18;    // trial-start pulse from the bench ESP32
}  // namespace pins

// ---------------------------------------------------------------------------
// C2 · Sign conventions
//   theta : wrist angle [rad], positive in EXTENSION, 0 = hand aligned
//   tau   : wrist torque [N·m], positive in EXTENSION
//   IMU frame: x along the forearm towards the hand, z dorsal, y = z × x.
//   Flexion/extension is the rotation about y. Both IMUs mounted identically.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// C3 · Data types (units: m/s², rad/s, rad, N·m, s)
// ---------------------------------------------------------------------------
struct ImuSample {
  uint32_t t_us;   // micros() at the start of the read
  float acc[3];    // [m/s²] x, y, z
  float gyr[3];    // [rad/s] x, y, z (bias removed)
};

// Gains are tabulated for a few tremor frequencies and interpolated by the
// adaptive estimator (J-4.1).
constexpr int N_F0 = 3;
constexpr int N_STATE = 4;  // x = [theta_v, omega_v, theta_t, omega_t]

struct Gains {
  float f0_hz[N_F0] = {4.0f, 5.0f, 6.0f};
  float K[N_F0][N_STATE] = {};  // LQR gains, tau = -K·x_hat
  // Kalman tuning (J-1.5)
  float q_voluntary = 1e-3f;
  float q_tremor = 1e-1f;
  float r_theta = 1e-2f;
  float r_omega = 1e-3f;
};

struct DebugState {
  float theta_hat = 0.0f;    // total wrist angle estimate [rad]
  float theta_v_hat = 0.0f;  // voluntary component [rad]
  float theta_t_hat = 0.0f;  // tremor component [rad]
  float f_hat = 0.0f;        // estimated tremor frequency [Hz]
  float tau_cmd = 0.0f;      // commanded wrist torque [N·m]
};

// ---------------------------------------------------------------------------
// C4 · Nominal mechanical parameters (replaced by measured values at S3)
// ---------------------------------------------------------------------------
namespace mech {
constexpr float J_HAND = 1.5e-3f;      // [kg·m²] hand + shell inertia
constexpr float K_WRIST = 0.3f;        // [N·m/rad] passive stiffness
constexpr float B_WRIST = 0.02f;       // [N·m·s/rad] damping
constexpr float R_PULLEY = 0.005f;     // [m] motor pulley radius
constexpr float R_WRIST = 0.030f;      // [m] cable lever arm at the wrist
constexpr float GEAR_N = R_WRIST / R_PULLEY;
constexpr float CABLE_PRETENSION_N = 5.0f;
constexpr float DELAY_S = 0.006f;      // sensor -> torque total delay
}  // namespace mech

// ---------------------------------------------------------------------------
// C5 · Firmware API (the boundary between the two code bases)
// ---------------------------------------------------------------------------
// Louis — src/drivers/
void motors_init();
void motors_set_torque(float tau_wrist_Nm);  // splits A/B, adds pretension, saturates
void motors_disable();
bool motors_enabled();
float motors_get_angle(int id);              // [rad], id = 0 (A) or 1 (B)
bool imu_init();
bool imu_read(ImuSample* s1, ImuSample* s2);
float battery_voltage();                     // [V]

// Joseph — src/control/
void controller_init(const Gains& g);
float controller_step(const ImuSample& s1, const ImuSample& s2, float dt);  // returns tau [N·m]
void controller_get_debug(DebugState* d);

// ---------------------------------------------------------------------------
// C6 · Safety limits (enforced in drivers and in the main loop)
// ---------------------------------------------------------------------------
namespace safety {
constexpr float TAU_MAX_NM = 0.3f;          // nominal software limit
constexpr float TAU_MAX_REDUCED_NM = 0.1f;  // "reduced torque" phase
constexpr float THETA_MAX_RAD = 0.6109f;    // 35 deg -> disable
constexpr uint32_t LOOP_OVERRUN_US = 3000;  // one iteration > 3 ms -> disable
constexpr int IMU_FAIL_MAX = 3;             // consecutive failed reads -> disable
}  // namespace safety

// ---------------------------------------------------------------------------
// Loop timing
// ---------------------------------------------------------------------------
constexpr uint32_t LOOP_HZ = 500;
constexpr uint32_t LOOP_PERIOD_US = 1000000 / LOOP_HZ;  // 2000 us
constexpr float LOOP_DT_S = 1.0f / LOOP_HZ;
