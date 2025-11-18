// IRadio.h
// Abstraction for outbound payload transmission with ACK tracking. App uses
// this interface while drivers::LoRaRadio provides the RadioLib-backed impl.

#pragma once

#include <string>
#include <cstdint>

namespace ports {

struct TxConfig {
  uint8_t maxAttempts;
  uint32_t ackTimeoutMs;
  uint32_t backoffMinMs;
  uint32_t backoffMaxMs;
};

struct TxResult {
  bool acked = false;
  std::string ackStatus;
};

class IRadio {
 public:
  virtual ~IRadio() = default;
  virtual TxResult transmit(const std::string& payload,
                            uint32_t counter,
                            const TxConfig& config) = 0;
};

}  // namespace ports
