// SystemClock.h
// Arduino-backed implementation of IClock using millis()/delay().

#pragma once

#include "ports/IClock.h"

namespace drivers {

class SystemClock : public ports::IClock {
 public:
  uint32_t nowMs() const override;
  void delayMs(uint32_t ms) override;
};

}  // namespace drivers
