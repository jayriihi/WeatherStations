// LoRaPacket.h
// Encapsulates parsing and validation of the Pearl → base LoRa payloads.
// Performs CRC16 checks and JSON parsing without depending on Arduino APIs.

#pragma once

#include <string>

#include "core/Types.h"

namespace domain {

enum class ParseStatus {
  kOk,
  kEmpty,
  kMissingTrailer,
  kTrailerNotHex,
  kCrcMismatch,
  kJsonError,
  kZeroPayload,
};

struct ParseResult {
  ParseStatus status = ParseStatus::kEmpty;
  core::ParsedPacket packet;
  bool ok() const { return status == ParseStatus::kOk; }
};

class LoRaPacket {
 public:
  ParseResult parse(const std::string& raw) const;
  static bool IsCrcFailure(ParseStatus status);
  static uint16_t ComputeCrc(const std::string& data);

 private:
  static uint16_t crc16(const uint8_t* data, size_t len, uint16_t crc = 0xFFFF);
};

}  // namespace domain
