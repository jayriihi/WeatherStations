// Config.h
// Centralized configuration for the Pearl station. Defines compile-time pin
// mappings, radio/battery settings, and scheduler timing constants shared
// by the app, drivers, and domain layers.

#pragma once

#include <cstdint>

namespace core {

struct RadioPins {
  int nss;
  int dio1;
  int rst;
  int busy;
  int sck;
  int miso;
  int mosi;
};

struct LoRaSettings {
  float frequencyMHz;
  float bandwidthKHz;
  uint8_t spreadingFactor;
  uint8_t codingRate;
  int8_t outputPowerDbm;
  uint16_t syncWord;
  bool crcEnabled;
};

struct WindsonicSettings {
  bool enabled;
  uint32_t baud;
  int rxPin;
  int txPin;
};

struct BatterySettings {
  bool enabled;
  int adcPin;
  float referenceVoltage;
  float maxCounts;
  float dividerGain;
  uint8_t samples;
};

struct AppTimings {
  uint32_t sampleIntervalMs;
  uint32_t blockDurationMs;
  uint8_t maxTxAttempts;
  uint32_t ackTimeoutMs;
  uint32_t ackBackoffMinMs;
  uint32_t ackBackoffMaxMs;
  uint32_t jitterMinMs;
  uint32_t jitterMaxMs;
};

constexpr RadioPins kRadioPins{8, 14, 12, 13, 9, 11, 10};
constexpr LoRaSettings kLoRaSettings{915.0f, 125.0f, 9, 5, 20, 0x34, true};
constexpr WindsonicSettings kWindsonicSettings{true, 4800, 19, -1};
constexpr BatterySettings kBatterySettings{false, -1, 3.3f, 4095.0f, 4.6f, 8};
constexpr AppTimings kAppTimings{1000, 120000, 3, 2500, 200, 499, 200, 599};

}  // namespace core
