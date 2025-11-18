// LoRaRadio.h
// Declares the RadioLib-backed driver that implements IRadio. Responsible for
// configuring the SX1262 modem, sending payloads, and handling ACK parsing so
// higher layers only deal with TxResult structures.

#pragma once

#include <RadioLib.h>

#include "core/Config.h"
#include "core/Result.h"
#include "ports/IRadio.h"

namespace drivers {

class LoRaRadio : public ports::IRadio {
 public:
  LoRaRadio(SX1262& radio, core::LoRaSettings settings);

  core::Result begin();
  ports::TxResult transmit(const std::string& payload,
                           uint32_t counter,
                           const ports::TxConfig& config) override;

 private:
  void configureRadio();
  bool waitForAck(uint32_t expectCnt, uint32_t timeoutMs, std::string* status);
  bool parseAckLine(const String& line, uint32_t& outCnt, String& outStatus);

  SX1262& radio_;
  core::LoRaSettings settings_;
};

}  // namespace drivers
