// Config.h
// Central location for hardware pins, credentials, and timing constants used
// by the base station firmware.

#pragma once

#include <cstddef>
#include <cstdint>

namespace core {

struct WifiCredential {
  const char* ssid;
  const char* password;
};

struct RadioPins {
  int nss;
  int dio1;
  int rst;
  int busy;
  int sck;
  int miso;
  int mosi;
};

struct AppTimings {
  uint32_t radioPollIntervalMs;
  uint32_t heartbeatBlinkIntervalMs;
  uint32_t heartbeatOnMs;
  uint32_t heartbeatHoldMs;
  uint32_t wifiRetryIntervalMs;
  uint32_t httpRetryDelayMs;
  uint32_t minPostIntervalMs;
  uint32_t httpDuplicateWindowMs;
  uint32_t rfDuplicateWindowMs;
  uint32_t inflightGuardMs;
};

constexpr WifiCredential kWifiNetworks[] = {
    {"Hapenny", "hapennyhouse"},
    {"Spanky’s House", "131Glenmont"},
    {"Jaysphone", "Riihiluoma"},
};
constexpr size_t kWifiNetworkCount = sizeof(kWifiNetworks) / sizeof(kWifiNetworks[0]);

constexpr RadioPins kRadioPins{8, 14, 12, 13, 9, 11, 10};
constexpr AppTimings kAppTimings{20, 2000, 50, 10000, 2000, 1200, 20000, 70000, 5000, 5000};

constexpr const char* kPostBase =
    "https://script.google.com/macros/s/AKfycbxkcVc6BP2oJtQcAB8cAvWWrIU9eDIGanyI5yWVj7GwHgISrCKnozDGZMXJobOxHGFu/exec";
constexpr const char* kApiKey = "jI6nrJ2KTsgK0SDu";
constexpr int kLedHeartbeatPin = 35;
constexpr bool kEnableTestPosts = false;

}  // namespace core
