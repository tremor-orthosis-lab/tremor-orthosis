// power.cpp — Battery voltage through the 100 k / 20 k divider on GPIO36
// (20 k: available in the resistor kit; 12.6 V full charge -> 2.1 V at the pin).

#include <Arduino.h>

#include "contracts.h"

float battery_voltage() {
  constexpr float R_TOP = 100e3f;
  constexpr float R_BOTTOM = 20e3f;
  const float v_pin = analogReadMilliVolts(pins::VBAT_ADC) / 1000.0f;
  return v_pin * (R_TOP + R_BOTTOM) / R_BOTTOM;
}
