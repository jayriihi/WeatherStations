// App.cpp
// Implements the App orchestrator that wires the scheduler to the wind sensor,
// averager, payload builder, and radio. Enforces timing, retries, and logging
// while staying independent of Arduino APIs via the port interfaces.

#include "app/App.h"

#include <algorithm>

#include "core/Logging.h"

namespace app {

App::App(ports::IWindSensor& windSensor,
         ports::IRadio& radio,
         ports::IClock& clock,
         ports::IPowerMonitor& powerMonitor,
         core::AppTimings timings)
    : windSensor_(windSensor),
      radio_(radio),
      clock_(clock),
      powerMonitor_(powerMonitor),
      timings_(timings) {}

void App::setup() { ensureTasks(); }

void App::loop(uint32_t nowMs) {
  ensureTasks();
  scheduler_.tick(nowMs);
}

void App::ensureTasks() {
  if (tasksInitialized_) {
    return;
  }

  sampleTaskId_ = scheduler_.schedule(
      timings_.sampleIntervalMs,
      [this](uint32_t now) { handleSample(now); });

  blockTaskId_ = scheduler_.schedule(
      timings_.blockDurationMs,
      [this](uint32_t now) { handleBlock(now); });

  statusTaskId_ = scheduler_.schedule(
      timings_.blockDurationMs / 4,
      [this](uint32_t now) { logStatus(now); });

  tasksInitialized_ = true;
}

void App::handleSample(uint32_t nowMs) {
  if (nowMs < samplingBlockedUntil_) {
    return;
  }

  domain::WindSample sample;
  if (!windSensor_.readSample(sample)) {
    return;
  }

  averager_.addSample(sample);
  core::Logger::info("sample %3u: spd=%.2f m/s dir=%.1f deg",
                     static_cast<unsigned>(averager_.sampleCount()),
                     sample.speedMs,
                     sample.directionDeg);
}

void App::handleBlock(uint32_t nowMs) {
  core::WindSummary summary;
  if (!averager_.finalize(summary)) {
    core::Logger::warn("No samples collected during block");
    return;
  }

  float vbatt = powerMonitor_.readBatteryVolts();
  uint32_t counter = bootMinutes(nowMs);
  std::string payload = payloadBuilder_.build(summary, vbatt, counter);

  ports::TxConfig txConfig{timings_.maxTxAttempts,
                           timings_.ackTimeoutMs,
                           timings_.ackBackoffMinMs,
                           timings_.ackBackoffMaxMs};
  ports::TxResult txResult = radio_.transmit(payload, counter, txConfig);

  float windAvgKt = summary.averageSpeedMs * core::kMsToKnots;
  float windMaxKt = summary.maxGustMs * core::kMsToKnots;

  core::Logger::info("----- BLOCK RESULT -----");
  core::Logger::info("avg   = %.2f m/s (%.1f kt)", summary.averageSpeedMs, windAvgKt);
  core::Logger::info("gust  = %.2f m/s (%.1f kt)", summary.maxGustMs, windMaxKt);
  core::Logger::info("dir   = %.1f deg", summary.averageDirectionDeg);
  core::Logger::info("cnt   = %lu", static_cast<unsigned long>(counter));
  core::Logger::info("batt  = %.2f V", vbatt);
  core::Logger::info("[TX] %s", payload.c_str());
  core::Logger::info("------------------------");

  if (txResult.acked) {
    core::Logger::info("PEARL: ACKed with status %s", txResult.ackStatus.c_str());
  } else {
    core::Logger::warn("PEARL: no ACK received after retries");
  }

  uint32_t jitter = timings_.jitterMinMs;
  if (timings_.jitterMaxMs > timings_.jitterMinMs) {
    uint32_t range = timings_.jitterMaxMs - timings_.jitterMinMs;
    jitter += clock_.random(0, range);
  }

  samplingBlockedUntil_ = nowMs + jitter;
  scheduler_.delayTask(blockTaskId_, jitter);
}

void App::logStatus(uint32_t /*nowMs*/) {
  core::Logger::info("status: block samples=%u",
                     static_cast<unsigned>(averager_.sampleCount()));
}

uint32_t App::bootMinutes(uint32_t nowMs) const {
  return nowMs / 60000UL;
}

}  // namespace app
