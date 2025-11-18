// IClock.h
// Port interface abstracting time and random jitter sources for the app.
// Implemented by drivers::SystemClock so scheduling logic remains testable.

#pragma once

#include <cstdint>

namespace ports {

class IClock {
 public:
  virtual ~IClock() = default;
  virtual uint32_t nowMs() const = 0;
  virtual uint32_t random(uint32_t min, uint32_t max) = 0;
};

}  // namespace ports
