#ifndef UNIT_TEST

#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>

#include <memory>

#include "app/App.h"
#include "core/Config.h"
#include "core/Logging.h"
#include "core/Result.h"
#include "drivers/BatteryMonitor.h"
#include "drivers/LoRaRadio.h"
#include "drivers/SystemClock.h"
#include "drivers/WindsonicSensor.h"

namespace {
Module* gModule = nullptr;
SX1262* gRadio = nullptr;
drivers::SystemClock gClock;
std::unique_ptr<drivers::WindsonicSensor> gWindSensor;
std::unique_ptr<drivers::LoRaRadio> gLoRaRadio;
std::unique_ptr<drivers::BatteryMonitor> gBatteryMonitor;
std::unique_ptr<app::App> gApp;

void initRadioHardware() {
  SPI.begin(core::kRadioPins.sck,
            core::kRadioPins.miso,
            core::kRadioPins.mosi,
            core::kRadioPins.nss);
  gModule = new Module(core::kRadioPins.nss,
                       core::kRadioPins.dio1,
                       core::kRadioPins.rst,
                       core::kRadioPins.busy);
  gRadio = new SX1262(gModule);
  gLoRaRadio.reset(new drivers::LoRaRadio(*gRadio, core::kLoRaSettings));
  core::Result radioInit = gLoRaRadio->begin();
  if (!radioInit.ok) {
    core::Logger::error("%s", radioInit.message.c_str());
    while (true) {
      delay(1000);
    }
  }
  core::Logger::info("PEARL: LoRa OK");
}

void initWindSensor() {
  gWindSensor.reset(new drivers::WindsonicSensor(Serial1, core::kWindsonicSettings));
  gWindSensor->begin();
  if (core::kWindsonicSettings.enabled) {
    core::Logger::info("WindSonic UART ready");
  } else {
    core::Logger::info("WindSonic simulator active");
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(300);
  core::Logger::begin(&Serial);
  core::Logger::info("PEARL booting");

  initRadioHardware();
  initWindSensor();

  gBatteryMonitor.reset(new drivers::BatteryMonitor(core::kBatterySettings));
  gApp.reset(new app::App(*gWindSensor, *gLoRaRadio, gClock, *gBatteryMonitor));
  gApp->setup();

  core::Logger::info("USE_WINDSONIC=%d BLOCK_SECONDS=%lu",
                     core::kWindsonicSettings.enabled ? 1 : 0,
                     static_cast<unsigned long>(core::kAppTimings.blockDurationMs / 1000UL));
}

void loop() {
  if (gApp) {
    gApp->loop(gClock.nowMs());
  }
  delay(5);
}

#endif  // UNIT_TEST
