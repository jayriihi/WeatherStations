// BatteryMonitor.cpp
// Implements the ADC-based power monitor for Pearl. Handles averaging several
// readings, applying divider gain, and returning a voltage (or sentinel) via
// the IPowerMonitor interface.

#include "drivers/BatteryMonitor.h"

#include <Arduino.h>

namespace drivers {

BatteryMonitor::BatteryMonitor(core::BatterySettings settings)
    : settings_(settings) {}

float BatteryMonitor::readBatteryVolts() {
  if (!settings_.enabled || settings_.adcPin < 0) {
    return -1.0f;
  }

  uint32_t total = 0;
  for (uint8_t i = 0; i < settings_.samples; ++i) {
    total += analogRead(settings_.adcPin);
    delay(2);
  }

  float avgCounts = static_cast<float>(total) / settings_.samples;
  float volts = (avgCounts / settings_.maxCounts) * settings_.referenceVoltage;
  volts *= settings_.dividerGain;
  return volts;
}

}  // namespace drivers
