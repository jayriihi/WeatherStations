// App.h
// High-level orchestrator for the base station firmware. Coordinates radio
// RX, ACKs, HTTP posting, WiFi connectivity, and LED heartbeats via ports.

#pragma once

#include <string>

#include "app/Scheduler.h"
#include "core/Config.h"
#include "core/Types.h"
#include "domain/AckLogic.h"
#include "domain/ConnectivityPolicy.h"
#include "domain/HttpPayloadBuilder.h"
#include "domain/LoRaPacket.h"
#include "ports/IClock.h"
#include "ports/IHttpClient.h"
#include "ports/IIndicator.h"
#include "ports/IRadio.h"
#include "ports/IWifi.h"

namespace app {

class App {
 public:
  App(ports::IRadio& radio,
      ports::IWifi& wifi,
      ports::IHttpClient& http,
      ports::IIndicator& indicator,
      ports::IClock& clock,
      core::AppTimings timings,
      uint32_t bootId);

  void setup();
  void loop(uint32_t nowMs);

 private:
  void ensureTasks();
  void pollRadio(uint32_t nowMs);
  void updateIndicator(uint32_t nowMs);
  void maintainWifi(uint32_t nowMs);
  void maybeSendTestPacket(uint32_t nowMs);

  void processFrame(const core::ReceivedFrame& frame, uint32_t nowMs);
  void handleParseFailure(domain::ParseStatus status);
  void sendAck(int32_t counter, const std::string& status);
  int32_t computeDropGap(int32_t counter);
  float computeDropRate(uint32_t deliveredNext) const;

  ports::IRadio& radio_;
  ports::IWifi& wifi_;
  ports::IHttpClient& http_;
  ports::IIndicator& indicator_;
  ports::IClock& clock_;
  core::AppTimings timings_;
  uint32_t bootId_;

  Scheduler scheduler_;
  domain::LoRaPacket packetParser_;
  domain::AckLogic ackLogic_;
  domain::HttpPayloadBuilder payloadBuilder_;
  domain::ConnectivityPolicy connectivityPolicy_;

  Scheduler::TaskId radioTaskId_ = 0;
  Scheduler::TaskId heartbeatTaskId_ = 0;
  Scheduler::TaskId wifiTaskId_ = 0;
  Scheduler::TaskId testTaskId_ = 0;
  bool tasksInitialized_ = false;

  uint32_t startMs_ = 0;
  uint32_t ledHoldUntilMs_ = 0;
  uint32_t lastHeartbeatMs_ = 0;
  bool heartbeatOn_ = false;
  uint32_t heartbeatOffMs_ = 0;

  std::string lastRfPayload_;
  uint32_t lastRfMs_ = 0;
  uint32_t rfDupCount_ = 0;
  uint32_t rfDupLastLogMs_ = 0;

  uint32_t crcOk_ = 0;
  uint32_t crcFail_ = 0;
  uint32_t rfDupTotal_ = 0;
  uint32_t deliveredTotal_ = 0;
  uint32_t dropTotal_ = 0;
  int32_t lastCntAccepted_ = -1;
  int32_t nominalCntStep_ = -1;

  bool enableTestPosts_ = core::kEnableTestPosts;
  uint32_t lastTestMinute_ = 0;
  uint32_t testCounter_ = 0;
};

}  // namespace app
