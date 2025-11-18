#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>
#include <WiFi.h>
#include <esp_system.h>
#include <time.h>

#include <memory>

#include "app/App.h"
#include "core/Config.h"
#include "core/Logging.h"
#include "drivers/HttpClient.h"
#include "drivers/LedIndicator.h"
#include "drivers/LoRaRadio.h"
#include "drivers/SystemClock.h"
#include "drivers/WifiManager.h"

namespace {
Module* gModule = nullptr;
SX1262* gRadio = nullptr;
drivers::SystemClock gClock;
std::unique_ptr<drivers::LedIndicator> gIndicator;
std::unique_ptr<drivers::WifiManager> gWifi;
std::unique_ptr<drivers::HttpClient> gHttp;
std::unique_ptr<drivers::LoRaRadio> gLoRa;
std::unique_ptr<app::App> gApp;
uint32_t gBootId = 0;

void initTimeUTC() {
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  Serial.print("NTP: syncing");
  for (int i = 0; i < 40; ++i) {
    if (time(nullptr) > 1600000000) {
      Serial.println("\nNTP OK");
      return;
    }
    Serial.print(".");
    delay(250);
  }
  Serial.println("\nNTP: failed (will still try).");
}

void bootIndicatorTest() {
  for (int i = 0; i < 3; ++i) {
    gIndicator->set(true);
    delay(150);
    gIndicator->set(false);
    delay(150);
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(300);
  core::Logger::begin(&Serial);

  gIndicator.reset(new drivers::LedIndicator(core::kLedHeartbeatPin));
  gIndicator->begin();
  bootIndicatorTest();

  gWifi.reset(new drivers::WifiManager(core::kWifiNetworks, core::kWifiNetworkCount));
  gWifi->connect(15000);
  initTimeUTC();

  SPI.begin(core::kRadioPins.sck,
            core::kRadioPins.miso,
            core::kRadioPins.mosi,
            core::kRadioPins.nss);
  gModule = new Module(core::kRadioPins.nss,
                       core::kRadioPins.dio1,
                       core::kRadioPins.rst,
                       core::kRadioPins.busy);
  gRadio = new SX1262(gModule);
  gLoRa.reset(new drivers::LoRaRadio(*gRadio));
  core::Result radioInit = gLoRa->begin();
  if (!radioInit.ok) {
    core::Logger::error("LoRa init failed: %s", radioInit.message.c_str());
    while (true) {
      delay(1000);
    }
  }

  gHttp.reset(new drivers::HttpClient(core::kPostBase, core::kApiKey));
  gBootId = esp_random();

  gApp.reset(new app::App(*gLoRa,
                          *gWifi,
                          *gHttp,
                          *gIndicator,
                          gClock,
                          core::kAppTimings,
                          gBootId));
  gApp->setup();
  core::Logger::info("BASE: ready (listening)");
}

void loop() {
  if (gApp) {
    gApp->loop(gClock.nowMs());
  }
  delay(5);
}
