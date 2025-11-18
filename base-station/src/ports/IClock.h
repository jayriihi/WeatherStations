// IClock.h
// Abstraction over timing utilities so the App/scheduler can run without
// depending on Arduino's millis().

#pragma once

#include <cstdint>

namespace ports {

class IClock {
 public:
  virtual ~IClock() = default;
  virtual uint32_t nowMs() const = 0;
  virtual void delayMs(uint32_t ms) = 0;
};

}  // namespace ports
