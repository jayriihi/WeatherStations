// LoRaRadio.h
// SX1262 driver implementing the IRadio interface for RX/TX + reset logic.

#pragma once

#include <RadioLib.h>

#include <string>

#include "core/Config.h"
#include "core/Result.h"
#include "ports/IRadio.h"

namespace drivers {

class LoRaRadio : public ports::IRadio {
 public:
  explicit LoRaRadio(SX1262& radio);

  core::Result begin() override;
  bool available() override;
 core::Result read(core::ReceivedFrame& frame) override;
 core::Result send(const std::string& payload) override;
 void resumeReceive() override;

 private:
  bool resetForReceive();

  SX1262& radio_;
};

}  // namespace drivers
