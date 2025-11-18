// LoRaPacket.cpp
// Implements CRC validation and JSON parsing for incoming Pearl packets.

#include "domain/LoRaPacket.h"

#include <ArduinoJson.h>
#include <cctype>
#include <cstdio>

namespace domain {

uint16_t LoRaPacket::crc16(const uint8_t* data, size_t len, uint16_t crc) {
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int b = 0; b < 8; ++b) {
      if (crc & 0x8000) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

ParseResult LoRaPacket::parse(const std::string& raw) const {
  ParseResult result;
  if (raw.empty()) {
    result.status = ParseStatus::kEmpty;
    return result;
  }

  auto star = raw.find_last_of('*');
  if (star == std::string::npos || raw.size() - star - 1 < 4) {
    result.status = ParseStatus::kMissingTrailer;
    return result;
  }

  char hex4[5];
  for (int i = 0; i < 4; ++i) {
    hex4[i] = raw[star + 1 + i];
    if (!std::isxdigit(static_cast<unsigned char>(hex4[i]))) {
      result.status = ParseStatus::kTrailerNotHex;
      return result;
    }
  }
  hex4[4] = '\0';

  unsigned rxCrcU = 0;
  std::sscanf(hex4, "%04X", &rxCrcU);
  uint16_t rxCrc = static_cast<uint16_t>(rxCrcU);

  std::string jsonPart = raw.substr(0, star);
  uint16_t calc = crc16(reinterpret_cast<const uint8_t*>(jsonPart.data()), jsonPart.size());
  if (calc != rxCrc) {
    result.status = ParseStatus::kCrcMismatch;
    return result;
  }

  StaticJsonDocument<256> doc;
  auto err = deserializeJson(doc, jsonPart);
  if (err) {
    result.status = ParseStatus::kJsonError;
    return result;
  }

  core::ParsedPacket packet;
  packet.windAvg = doc["wind_avg"] | 0.0f;
  packet.windMax = doc["wind_max"] | 0.0f;
  packet.windDir = doc["wind_dir"] | 0;
  packet.counter = doc["cnt"] | -1;
  packet.batteryVolts = doc["batt"] | -1.0f;

  if (packet.windAvg == 0.0f && packet.windMax == 0.0f && packet.windDir == 0) {
    result.status = ParseStatus::kZeroPayload;
    return result;
  }

  result.packet = packet;
  result.status = ParseStatus::kOk;
  return result;
}

bool LoRaPacket::IsCrcFailure(ParseStatus status) {
  return status == ParseStatus::kMissingTrailer ||
         status == ParseStatus::kTrailerNotHex ||
         status == ParseStatus::kCrcMismatch;
}

uint16_t LoRaPacket::ComputeCrc(const std::string& data) {
  return crc16(reinterpret_cast<const uint8_t*>(data.data()), data.size());
}

}  // namespace domain
