// BatteryMonitor.h
// Declares the hardware driver that implements IPowerMonitor by sampling the
// configured ADC pin and scaling counts into volts. Keeps ADC logic out of
// the App and domain layers.

#pragma once

#include "core/Config.h"
#include "ports/IPowerMonitor.h"

namespace drivers {

class BatteryMonitor : public ports::IPowerMonitor {
 public:
  explicit BatteryMonitor(core::BatterySettings settings);
  float readBatteryVolts() override;

 private:
  core::BatterySettings settings_;
};

}  // namespace drivers
