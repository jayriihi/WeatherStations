// LedIndicator.h
// Controls the onboard LED heartbeat indicator via digitalWrite.

#pragma once

#include "ports/IIndicator.h"

namespace drivers {

class LedIndicator : public ports::IIndicator {
 public:
  explicit LedIndicator(int pin);
  void begin();
  void set(bool on) override;

 private:
  int pin_;
};

}  // namespace drivers
