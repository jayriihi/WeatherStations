// Types.h
// Shared type definitions for the base station firmware. Keeps simple POD
// structs for parsed packets, radio frames, and statistics so layers can
// exchange data without Arduino dependencies.

#pragma once

#include <cstdint>
#include <string>

namespace core {

struct ReceivedFrame {
  std::string payload;
  float rssi = 0.0f;
  float snr = 0.0f;
  uint32_t receivedMs = 0;
};

struct ParsedPacket {
  float windAvg = 0.0f;
  float windMax = 0.0f;
  int windDir = 0;
  int32_t counter = -1;
  float batteryVolts = -1.0f;
};

struct HealthCounters {
  uint32_t crcOk = 0;
  uint32_t crcFail = 0;
  uint32_t rfDuplicate = 0;
  uint32_t uptimeSeconds = 0;
  uint32_t bootId = 0;
  int32_t dropGap = 0;
  uint32_t dropTotal = 0;
  float dropRate = 0.0f;
};

struct HttpPostRequest {
  ParsedPacket packet;
  HealthCounters health;
  float rssi = 0.0f;
  float snr = 0.0f;
};

}  // namespace core
