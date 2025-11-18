// IRadio.h
// Interface for LoRa radio operations (RX/TX) so higher layers stay hardware-free.

#pragma once

#include <string>

#include "core/Result.h"
#include "core/Types.h"

namespace ports {

class IRadio {
 public:
  virtual ~IRadio() = default;
  virtual core::Result begin() = 0;
  virtual bool available() = 0;
  virtual core::Result read(core::ReceivedFrame& frame) = 0;
  virtual core::Result send(const std::string& payload) = 0;
  virtual void resumeReceive() = 0;
};

}  // namespace ports
