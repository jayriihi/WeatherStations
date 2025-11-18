// IPowerMonitor.h
// Pure interface for reading battery voltage or other power metrics.
// Implemented by drivers::BatteryMonitor so the app can remain hardware-free.

#pragma once

namespace ports {

class IPowerMonitor {
 public:
  virtual ~IPowerMonitor() = default;
  virtual float readBatteryVolts() = 0;
};

}  // namespace ports
