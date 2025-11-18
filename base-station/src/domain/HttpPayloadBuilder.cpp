// HttpPayloadBuilder.cpp
// Formats the HTTP POST body exactly as the legacy base firmware expects.

#include "domain/HttpPayloadBuilder.h"

#include <cstdio>

#include "core/Types.h"

namespace domain {

std::string HttpPayloadBuilder::build(const core::HttpPostRequest& request) const {
  char buf[512];
  std::snprintf(buf,
                sizeof(buf),
                "wind_avg=%.1f&wind_max=%.1f&wind_dir=%d&cnt=%ld&batt=%.2f&"
                "rssi=%.1f&snr=%.1f&crc_ok=%lu&crc_fail=%lu&rf_dup=%lu&"
                "uptime_s=%lu&boot_id=%lu&fw=base_1&drop_gap=%ld&drop_total=%lu&drop_rate=%.4f",
                request.packet.windAvg,
                request.packet.windMax,
                request.packet.windDir,
                static_cast<long>(request.packet.counter),
                request.packet.batteryVolts,
                request.rssi,
                request.snr,
                static_cast<unsigned long>(request.health.crcOk),
                static_cast<unsigned long>(request.health.crcFail),
                static_cast<unsigned long>(request.health.rfDuplicate),
                static_cast<unsigned long>(request.health.uptimeSeconds),
                static_cast<unsigned long>(request.health.bootId),
                static_cast<long>(request.health.dropGap),
                static_cast<unsigned long>(request.health.dropTotal),
                request.health.dropRate);
  return std::string(buf);
}

}  // namespace domain
