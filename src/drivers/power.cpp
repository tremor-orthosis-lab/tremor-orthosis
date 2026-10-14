// power.cpp — Battery voltage through the 100 k / 22 k divider on GPIO36.

#include <Arduino.h>

#include "contracts.h"

float battery_voltage() {
  constexpr float R_TOP = 100e3f;
  constexpr float R_BOTTOM = 22e3f;
  const float v_pin = analogReadMilliVolts(pins::VBAT_ADC) / 1000.0f;
  return v_pin * (R_TOP + R_BOTTOM) / R_BOTTOM;
}
