// LoRaRadio.cpp
// Hardware driver for the SX1262 using the RadioLib library.

#include "drivers/LoRaRadio.h"

#include <Arduino.h>

#include "core/Logging.h"

namespace drivers {

LoRaRadio::LoRaRadio(SX1262& radio) : radio_(radio) {}

core::Result LoRaRadio::begin() {
  if (!resetForReceive()) {
    return core::Result::Failure("LoRa init failed");
  }
  return core::Result::Success();
}

bool LoRaRadio::available() { return radio_.getPacketLength() != 0; }

core::Result LoRaRadio::read(core::ReceivedFrame& frame) {
  String payload;
  int st = radio_.readData(payload);
  if (st != RADIOLIB_ERR_NONE) {
    return core::Result::Failure("RadioLib RX error " + std::to_string(st));
  }
  payload.trim();
  frame.payload = payload.c_str();
  frame.rssi = radio_.getRSSI();
  frame.snr = radio_.getSNR();
  frame.receivedMs = millis();
  return core::Result::Success();
}

core::Result LoRaRadio::send(const std::string& payload) {
  radio_.standby();
  String body(payload.c_str());
  int st = radio_.transmit(body);
  if (st != RADIOLIB_ERR_NONE) {
    resetForReceive();
    return core::Result::Failure("RadioLib TX error " + std::to_string(st));
  }
  resetForReceive();
  return core::Result::Success();
}

void LoRaRadio::resumeReceive() { resetForReceive(); }

bool LoRaRadio::resetForReceive() {
  radio_.standby();
  delay(5);
  int st = radio_.begin(915.0);
  if (st != RADIOLIB_ERR_NONE) {
    core::Logger::error("LoRa begin()-> %d", st);
    return false;
  }
  radio_.setDio2AsRfSwitch(true);
  radio_.setSyncWord(0x34);
  radio_.setBandwidth(125.0);
  radio_.setSpreadingFactor(9);
  radio_.setCodingRate(5);
  radio_.setRxBoostedGainMode(true);
  radio_.setCRC(true);
  radio_.setOutputPower(-10);
  radio_.startReceive();
  return true;
}

}  // namespace drivers
