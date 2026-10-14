// main.cpp — 500 Hz control loop, safety watchdog, serial commands, CSV log.
//
// Loop: a hardware timer (esp_timer, 2 ms) wakes the loop; each tick reads
// both IMUs, calls controller_step(), sends the torque to the motors and
// optionally logs one CSV line (format C3).
//
// Serial commands (921600 baud, one per line):
//   ARM / DISARM            enable / disable the motors
//   LOG on / LOG off        start / stop the CSV stream
//   SET tau_max <N·m>       software torque limit (<= 0.3)
//   SET ol_amp <N·m>        open-loop amplitude   (env "openloop" only)
//   SET ol_freq <Hz>        open-loop frequency   (env "openloop" only)
//   STATUS                  print state

#include <Arduino.h>
#include <esp_timer.h>

#include "contracts.h"

void motors_enable();  // provided by the motor driver (real or mock)

#ifdef USE_OPENLOOP_TEST
extern float g_openloop_amp_Nm;
extern float g_openloop_freq_Hz;
#endif

namespace {

volatile bool tick = false;
bool logging = false;
float tau_max = safety::TAU_MAX_REDUCED_NM;  // start in reduced-torque mode
int imu_fail_count = 0;
uint32_t overruns = 0;
const char* last_fault = "none";

void IRAM_ATTR on_timer(void*) { tick = true; }

void fault(const char* why) {
  motors_disable();
  last_fault = why;
  Serial.printf("# FAULT: %s -> motors disabled\n", why);
}

void print_header() {
  Serial.println(
      "t_us,ax1,ay1,az1,gx1,gy1,gz1,ax2,ay2,az2,gx2,gy2,gz2,"
      "theta_hat,theta_v_hat,theta_t_hat,f_hat,tau_cmd,vbat");
}

void log_line(const ImuSample& a, const ImuSample& b, const DebugState& d, float vbat) {
  Serial.printf(
      "%lu,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,"
      "%.4f,%.4f,%.4f,%.2f,%.4f,%.2f\n",
      (unsigned long)a.t_us, a.acc[0], a.acc[1], a.acc[2], a.gyr[0], a.gyr[1], a.gyr[2],
      b.acc[0], b.acc[1], b.acc[2], b.gyr[0], b.gyr[1], b.gyr[2], d.theta_hat, d.theta_v_hat,
      d.theta_t_hat, d.f_hat, d.tau_cmd, vbat);
}

void handle_command(String line) {
  line.trim();
  if (line.equalsIgnoreCase("ARM")) {
    motors_enable();
    Serial.printf("# ARMED, tau_max = %.3f N·m\n", tau_max);
  } else if (line.equalsIgnoreCase("DISARM")) {
    motors_disable();
    Serial.println("# DISARMED");
  } else if (line.equalsIgnoreCase("LOG on")) {
    logging = true;
    print_header();
  } else if (line.equalsIgnoreCase("LOG off")) {
    logging = false;
  } else if (line.equalsIgnoreCase("STATUS")) {
    Serial.printf("# armed=%d tau_max=%.3f overruns=%lu last_fault=%s vbat=%.2f\n",
                  motors_enabled(), tau_max, (unsigned long)overruns, last_fault,
                  battery_voltage());
  } else if (line.startsWith("SET ")) {
    const int sp = line.indexOf(' ', 4);
    if (sp < 0) {
      Serial.println("# usage: SET <key> <value>");
      return;
    }
    const String key = line.substring(4, sp);
    const float val = line.substring(sp + 1).toFloat();
    if (key == "tau_max") {
      tau_max = constrain(val, 0.0f, safety::TAU_MAX_NM);
#ifdef USE_OPENLOOP_TEST
    } else if (key == "ol_amp") {
      g_openloop_amp_Nm = val;
    } else if (key == "ol_freq") {
      g_openloop_freq_Hz = val;
#endif
    } else {
      Serial.printf("# unknown key: %s\n", key.c_str());
      return;
    }
    Serial.printf("# %s = %.4f\n", key.c_str(), val);
  } else if (line.length() > 0) {
    Serial.printf("# unknown command: %s\n", line.c_str());
  }
}

void poll_serial() {
  static String buf;
  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (buf.length()) handle_command(buf);
      buf = "";
    } else if (buf.length() < 64) {
      buf += c;
    }
  }
}

}  // namespace

void setup() {
  Serial.setTxBufferSize(4096);
  Serial.begin(921600);
  pinMode(pins::ESTOP, INPUT_PULLUP);
  pinMode(pins::BENCH_SYNC, INPUT);

  motors_init();  // always start disabled (C6)
  Serial.println("# tremor-orthosis: keep the hand still, calibrating gyros (2 s)...");
  if (!imu_init()) {
    Serial.println("# FAULT: IMU init failed (check wiring and addresses 0x68/0x69)");
  }
  controller_init(Gains{});

  esp_timer_create_args_t args = {};
  args.callback = &on_timer;
  args.name = "ctrl";
  esp_timer_handle_t timer;
  esp_timer_create(&args, &timer);
  esp_timer_start_periodic(timer, LOOP_PERIOD_US);
  Serial.println("# ready. Commands: ARM, DISARM, LOG on, LOG off, SET, STATUS");
}

void loop() {
  poll_serial();
  if (!tick) return;
  tick = false;

  const uint32_t t0 = micros();

  if (digitalRead(pins::ESTOP) == LOW && motors_enabled()) fault("emergency stop");

  ImuSample s1, s2;
  if (!imu_read(&s1, &s2)) {
    if (++imu_fail_count >= safety::IMU_FAIL_MAX) fault("IMU read failed 3x");
    return;
  }
  imu_fail_count = 0;

  float tau = controller_step(s1, s2, LOOP_DT_S);
  tau = constrain(tau, -tau_max, tau_max);

  DebugState d;
  controller_get_debug(&d);
  if (fabsf(d.theta_hat) > safety::THETA_MAX_RAD && motors_enabled()) fault("|theta| > 35 deg");

  motors_set_torque(tau);

  if (logging) log_line(s1, s2, d, battery_voltage());

  if (micros() - t0 > safety::LOOP_OVERRUN_US) {
    ++overruns;
    if (motors_enabled()) fault("loop overrun > 3 ms");
  }
}
