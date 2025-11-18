// PayloadBuilder.cpp
// Implements the JSON and CRC16 payload creation used for LoRa transmissions.
// Converts averaged wind measurements and metadata into the exact string the
// rest of the system transmits.

#include "domain/PayloadBuilder.h"

#include <cstdint>
#include <cstdio>

#include "core/Types.h"

namespace {
uint16_t crc16_ccitt(const uint8_t* data, size_t len, uint16_t crc = 0xFFFF) {
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
}  // namespace

namespace domain {

std::string PayloadBuilder::build(const core::WindSummary& summary,
                                  float batteryVolts,
                                  uint32_t counter) const {
  float windAvgKt = summary.averageSpeedMs * core::kMsToKnots;
  float windMaxKt = summary.maxGustMs * core::kMsToKnots;

  char body[160];
  std::snprintf(body, sizeof(body),
                "{\"wind_avg\":%.1f,\"wind_max\":%.1f,\"wind_dir\":%.0f,"
                "\"cnt\":%lu,\"batt\":%.2f}",
                windAvgKt,
                windMaxKt,
                summary.averageDirectionDeg,
                static_cast<unsigned long>(counter),
                batteryVolts);

  std::string payload(body);
  uint16_t crc = crc16_ccitt(reinterpret_cast<const uint8_t*>(payload.data()),
                             payload.size());
  char trailer[6];
  std::snprintf(trailer, sizeof(trailer), "*%04X", static_cast<unsigned>(crc));
  payload += trailer;
  return payload;
}

}  // namespace domain
