// App.h
// High-level orchestration for the Pearl station. Owns the scheduler, domain
// services, and driver ports to coordinate sampling, averaging, payload
// generation, and LoRa transmissions.

#pragma once

#include <string>

#include "app/Scheduler.h"
#include "core/Config.h"
#include "core/Types.h"
#include "domain/PayloadBuilder.h"
#include "domain/WindAverager.h"
#include "ports/IClock.h"
#include "ports/IPowerMonitor.h"
#include "ports/IRadio.h"
#include "ports/IWindSensor.h"

namespace app {

class App {
 public:
  App(ports::IWindSensor& windSensor,
      ports::IRadio& radio,
      ports::IClock& clock,
      ports::IPowerMonitor& powerMonitor,
      core::AppTimings timings = core::kAppTimings);

  void setup();
  void loop(uint32_t nowMs);

 private:
  void ensureTasks();
  void handleSample(uint32_t nowMs);
  void handleBlock(uint32_t nowMs);
  void logStatus(uint32_t nowMs);
  core::WindSummary finalizeSummary();
  uint32_t bootMinutes(uint32_t nowMs) const;

  ports::IWindSensor& windSensor_;
  ports::IRadio& radio_;
  ports::IClock& clock_;
  ports::IPowerMonitor& powerMonitor_;
  core::AppTimings timings_;

  Scheduler scheduler_;
  domain::WindAverager averager_;
  domain::PayloadBuilder payloadBuilder_;

  Scheduler::TaskId sampleTaskId_ = 0;
  Scheduler::TaskId blockTaskId_ = 0;
  Scheduler::TaskId statusTaskId_ = 0;
  bool tasksInitialized_ = false;
  uint32_t samplingBlockedUntil_ = 0;
};

}  // namespace app
