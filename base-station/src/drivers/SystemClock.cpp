// SystemClock.cpp
// Wraps Arduino timing primitives for the modular base station.

#include "drivers/SystemClock.h"

#include <Arduino.h>

namespace drivers {

uint32_t SystemClock::nowMs() const { return millis(); }

void SystemClock::delayMs(uint32_t ms) { delay(ms); }

}  // namespace drivers
