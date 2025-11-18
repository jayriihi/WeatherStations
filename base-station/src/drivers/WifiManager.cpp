// WifiManager.cpp
// Connects to predefined WiFi networks, mimicking the legacy behavior.

#include "drivers/WifiManager.h"

#include <Arduino.h>
#include <WiFi.h>

#include "core/Logging.h"

namespace drivers {

WifiManager::WifiManager(const core::WifiCredential* creds, std::size_t count)
    : creds_(creds), count_(count) {}

void WifiManager::connect(uint32_t maxWaitMs) {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  WiFi.mode(WIFI_STA);
  uint32_t perNetWait = (count_ > 0) ? maxWaitMs / count_ : maxWaitMs;
  if (perNetWait < 2000) {
    perNetWait = 3000;
  }

  for (std::size_t i = 0; i < count_; ++i) {
    const auto& cred = creds_[i];
    core::Logger::info("WiFi: trying SSID '%s'", cred.ssid);
    WiFi.begin(cred.ssid, cred.password);

    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < perNetWait) {
      delay(250);
    }

    if (WiFi.status() == WL_CONNECTED) {
      core::Logger::info("WiFi OK ssid=%s ip=%s",
                         cred.ssid,
                         WiFi.localIP().toString().c_str());
      return;
    }

    core::Logger::warn("WiFi: failed on this SSID, trying next");
    WiFi.disconnect(true, true);
    delay(200);
  }

  core::Logger::warn("WiFi: not connected to any configured network, continuing offline");
}

bool WifiManager::connected() const {
  return WiFi.status() == WL_CONNECTED;
}

}  // namespace drivers
