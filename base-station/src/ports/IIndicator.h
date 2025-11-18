// IIndicator.h
// Simple interface for driving the base station LED indicator/heartbeat.

#pragma once

namespace ports {

class IIndicator {
 public:
  virtual ~IIndicator() = default;
  virtual void set(bool on) = 0;
};

}  // namespace ports
