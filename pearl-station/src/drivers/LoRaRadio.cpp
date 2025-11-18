// LoRaRadio.cpp
// Concrete hardware driver for the SX1262 radio. Wraps RadioLib calls to
// initialize the modem, transmit payloads, and wait for ACKs while exposing
// the clean IRadio interface back to App.

#include "drivers/LoRaRadio.h"

#include <Arduino.h>
#include <esp_system.h>

#include <cstdio>
#include <string>

#include "core/Logging.h"

namespace drivers {

LoRaRadio::LoRaRadio(SX1262& radio, core::LoRaSettings settings)
    : radio_(radio), settings_(settings) {}

core::Result LoRaRadio::begin() {
  int st = radio_.begin(settings_.frequencyMHz);
  if (st != RADIOLIB_ERR_NONE) {
    return core::Result::Failure("LoRa init failed: " + std::to_string(st));
  }

  configureRadio();
  return core::Result::Success();
}

void LoRaRadio::configureRadio() {
  radio_.setDio2AsRfSwitch(true);
  radio_.setSyncWord(settings_.syncWord);
  radio_.setBandwidth(settings_.bandwidthKHz);
  radio_.setSpreadingFactor(settings_.spreadingFactor);
  radio_.setCodingRate(settings_.codingRate);
  radio_.setOutputPower(settings_.outputPowerDbm);
  radio_.setCRC(settings_.crcEnabled);
}

ports::TxResult LoRaRadio::transmit(const std::string& payload,
                                    uint32_t counter,
                                    const ports::TxConfig& config) {
  ports::TxResult result;
  String body(payload.c_str());

  uint8_t attempt = 0;
  while (attempt < config.maxAttempts) {
    attempt++;
    core::Logger::info("PEARL: TX attempt %u", static_cast<unsigned>(attempt));
    int st = radio_.transmit(body);
    if (st != RADIOLIB_ERR_NONE) {
      core::Logger::error("LoRa transmit error %d", st);
    }

    std::string ackStatus;
    if (waitForAck(counter, config.ackTimeoutMs, &ackStatus)) {
      result.acked = true;
      result.ackStatus = ackStatus;
      break;
    }

    uint32_t backoff = config.backoffMinMs;
    if (config.backoffMaxMs > config.backoffMinMs) {
      uint32_t range = config.backoffMaxMs - config.backoffMinMs;
      backoff += esp_random() % (range + 1);
    }
    delay(backoff);
  }

  return result;
}

bool LoRaRadio::waitForAck(uint32_t expectCnt,
                           uint32_t timeoutMs,
                           std::string* status) {
  unsigned long t0 = millis();

  radio_.startReceive();
  unsigned long purgeEnd = t0 + 120;
  while (millis() < purgeEnd) {
    if (radio_.getPacketLength() != 0) {
      String junk;
      radio_.readData(junk);
    }
    delay(5);
  }

  radio_.startReceive();
  unsigned nonAckPrinted = 0;

  while ((millis() - t0) < timeoutMs) {
    if (radio_.getPacketLength() != 0) {
      String line;
      int st = radio_.readData(line);
      if (st == RADIOLIB_ERR_NONE) {
        line.trim();
        if (line.startsWith("ACK ")) {
          core::Logger::info("PEARL: ACK RX raw: %s", line.c_str());
        } else if (nonAckPrinted < 2) {
          core::Logger::info("PEARL: non-ACK sample: %s", line.c_str());
          nonAckPrinted++;
        }

        uint32_t cntRx = 0;
        String ackStatus;
        if (parseAckLine(line, cntRx, ackStatus) && cntRx == expectCnt) {
          if (status) {
            *status = ackStatus.c_str();
          }
          core::Logger::info("PEARL: got ACK cnt=%lu status=%s",
                              static_cast<unsigned long>(cntRx),
                              ackStatus.c_str());
          return true;
        }
      }
    }
    delay(10);
  }
  return false;
}

bool LoRaRadio::parseAckLine(const String& line,
                             uint32_t& outCnt,
                             String& outStatus) {
  int star = line.lastIndexOf('*');
  if (star < 0 || (line.length() - star - 1) < 4) {
    return false;
  }

  String head = line.substring(0, star);
  char hex4[5];
  hex4[0] = line[star + 1];
  hex4[1] = line[star + 2];
  hex4[2] = line[star + 3];
  hex4[3] = line[star + 4];
  hex4[4] = '\0';

  unsigned rx_u = 0;
  sscanf(hex4, "%04X", &rx_u);
  uint16_t calc = 0xFFFF;
  const uint8_t* data = reinterpret_cast<const uint8_t*>(head.c_str());
  size_t len = head.length();
  for (size_t i = 0; i < len; ++i) {
    calc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int b = 0; b < 8; ++b) {
      if (calc & 0x8000) {
        calc = static_cast<uint16_t>((calc << 1) ^ 0x1021);
      } else {
        calc <<= 1;
      }
    }
  }
  if (static_cast<uint16_t>(rx_u) != calc) {
    return false;
  }

  if (!head.startsWith("ACK ")) {
    return false;
  }
  int sp1 = head.indexOf(' ', 4);
  if (sp1 < 0) {
    return false;
  }

  String cntStr = head.substring(4, sp1);
  String stStr = head.substring(sp1 + 1);
  outCnt = static_cast<uint32_t>(cntStr.toInt());
  outStatus = stStr;
  return true;
}

}  // namespace drivers
