// SystemClock.h
// Declares the Arduino-backed implementation of IClock. Provides millis-based
// timestamps and RNG jitter so the App scheduler can work independently of
// Arduino globals.

#pragma once

#include "ports/IClock.h"

namespace drivers {

class SystemClock : public ports::IClock {
 public:
  uint32_t nowMs() const override;
  uint32_t random(uint32_t min, uint32_t max) override;
};

}  // namespace drivers
