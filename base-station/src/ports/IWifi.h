// IWifi.h
// Interface abstracting WiFi connectivity so App can request connections
// without hard dependencies on the Arduino WiFi stack.

#pragma once

#include <cstdint>

namespace ports {

class IWifi {
 public:
  virtual ~IWifi() = default;
  virtual void connect(uint32_t maxWaitMs) = 0;
  virtual bool connected() const = 0;
};

}  // namespace ports
