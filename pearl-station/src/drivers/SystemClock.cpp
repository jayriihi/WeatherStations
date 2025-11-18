// SystemClock.cpp
// Implements the real-time clock driver using Arduino's millis() and ESP32
// RNG. Supplies time and random offsets to the App via the IClock interface.

#include "drivers/SystemClock.h"

#include <Arduino.h>
#include <esp_system.h>

namespace drivers {

uint32_t SystemClock::nowMs() const { return millis(); }

uint32_t SystemClock::random(uint32_t min, uint32_t max) {
  if (max <= min) {
    return min;
  }
  uint32_t span = max - min + 1;
  uint32_t value = esp_random() % span;
  return min + value;
}

}  // namespace drivers
