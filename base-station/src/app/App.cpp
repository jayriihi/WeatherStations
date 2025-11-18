// App.cpp
// Implements the orchestrator that ties together radio RX, ACKs, WiFi/HTTP,
// and LED indicators for the base station firmware.

#include "app/App.h"

#include <cstdio>
#include <ctime>

#include "core/Logging.h"

namespace {
constexpr uint32_t kRfDupLogIntervalMs = 20000UL;
constexpr time_t kValidTimeEpoch = 1600000000;
}

namespace app {

App::App(ports::IRadio& radio,
         ports::IWifi& wifi,
         ports::IHttpClient& http,
         ports::IIndicator& indicator,
         ports::IClock& clock,
         core::AppTimings timings,
         uint32_t bootId)
    : radio_(radio),
      wifi_(wifi),
      http_(http),
      indicator_(indicator),
      clock_(clock),
      timings_(timings),
      bootId_(bootId),
      ackLogic_(timings.inflightGuardMs, timings.minPostIntervalMs, timings.httpDuplicateWindowMs),
      connectivityPolicy_(timings.wifiRetryIntervalMs) {}

void App::setup() {
  startMs_ = clock_.nowMs();
  ensureTasks();
}

void App::loop(uint32_t nowMs) {
  ensureTasks();
  scheduler_.tick(nowMs);
}

void App::ensureTasks() {
  if (tasksInitialized_) {
    return;
  }

  startMs_ = clock_.nowMs();

  radioTaskId_ = scheduler_.schedule(
      timings_.radioPollIntervalMs,
      [this](uint32_t now) { pollRadio(now); });

  heartbeatTaskId_ = scheduler_.schedule(
      10,
      [this](uint32_t now) { updateIndicator(now); });

  wifiTaskId_ = scheduler_.schedule(
      timings_.wifiRetryIntervalMs,
      [this](uint32_t now) { maintainWifi(now); });

  if (enableTestPosts_) {
    testTaskId_ = scheduler_.schedule(
        1000,
        [this](uint32_t now) { maybeSendTestPacket(now); });
  }

  tasksInitialized_ = true;
}

void App::pollRadio(uint32_t nowMs) {
  ackLogic_.expireInflight(nowMs);
  if (!radio_.available()) {
    return;
  }

  core::ReceivedFrame frame;
  core::Result rx = radio_.read(frame);
  if (!rx.ok) {
    core::Logger::error("BASE RX error: %s", rx.message.c_str());
    radio_.resumeReceive();
    return;
  }

  if (!frame.payload.empty() && frame.payload == lastRfPayload_ &&
      (nowMs - lastRfMs_) < timings_.rfDuplicateWindowMs) {
    rfDupCount_++;
    rfDupTotal_++;
    if ((nowMs - rfDupLastLogMs_) > kRfDupLogIntervalMs) {
      core::Logger::info("BASE: duplicate RF payload burst (+%u skipped)", rfDupCount_);
      rfDupLastLogMs_ = nowMs;
      rfDupCount_ = 0;
    }
    radio_.resumeReceive();
    return;
  }

  if (rfDupCount_ > 0) {
    core::Logger::info("BASE: duplicate burst ended (total skipped=%u)", rfDupCount_);
    rfDupCount_ = 0;
  }

  lastRfPayload_ = frame.payload;
  lastRfMs_ = nowMs;

  auto parseResult = packetParser_.parse(frame.payload);
  if (!parseResult.ok()) {
    handleParseFailure(parseResult.status);
    if (domain::LoRaPacket::IsCrcFailure(parseResult.status)) {
      crcFail_++;
    }
    radio_.resumeReceive();
    return;
  }

  crcOk_++;
  ledHoldUntilMs_ = nowMs + timings_.heartbeatHoldMs;

  core::ParsedPacket packet = parseResult.packet;
  if (packet.counter >= 0) {
    sendAck(packet.counter, "OK");
  }

  int32_t dropGap = computeDropGap(packet.counter);
  uint32_t uptimeSeconds = (clock_.nowMs() - startMs_) / 1000UL;
  uint32_t deliveredNext = deliveredTotal_ + 1;
  float dropRate = computeDropRate(deliveredNext);

  core::HealthCounters health;
  health.crcOk = crcOk_;
  health.crcFail = crcFail_;
  health.rfDuplicate = rfDupTotal_;
  health.uptimeSeconds = uptimeSeconds;
  health.bootId = bootId_;
  health.dropGap = dropGap;
  health.dropTotal = dropTotal_;
  health.dropRate = dropRate;

  core::HttpPostRequest request;
  request.packet = packet;
  request.health = health;
  request.rssi = frame.rssi;
  request.snr = frame.snr;
  std::string body = payloadBuilder_.build(request);

  auto decision = ackLogic_.evaluate(packet.counter, body, nowMs);
  if (!decision.shouldPost) {
    if (decision.sendAck) {
      sendAck(packet.counter, decision.status);
    }
    radio_.resumeReceive();
    return;
  }

  if (!wifi_.connected()) {
    wifi_.connect(2000);
  }

  ackLogic_.markInflight(packet.counter, nowMs);
  core::Logger::info("BASE POSTING (rssi=%.1f, snr=%.1f): %s",
                     frame.rssi,
                     frame.snr,
                     body.c_str());

  int code = http_.post(body);
  bool transient = (code < 0) || code == 408 || (code >= 500 && code < 600);
  if (transient) {
    core::Logger::warn("POST transient (code=%d), retrying once...", code);
    clock_.delayMs(timings_.httpRetryDelayMs);
    code = http_.post(body);
  } else if (code >= 400 && code < 500 && code != 400) {
    core::Logger::warn("Client error (%d), skipping retry", code);
  }

  if (code == 200 || code == 201 || code == 400) {
    deliveredTotal_++;
    if (packet.counter >= 0) {
      sendAck(packet.counter, "OK");
    }
    ackLogic_.markSuccess(packet.counter, body, nowMs);
  } else {
    core::Logger::warn("POST not successful (code=%d)", code);
    ackLogic_.expireInflight(nowMs);
  }

  radio_.resumeReceive();
}

void App::updateIndicator(uint32_t nowMs) {
  if (nowMs < ledHoldUntilMs_) {
    indicator_.set(true);
    heartbeatOn_ = false;
    return;
  }

  if (!heartbeatOn_ && (nowMs - lastHeartbeatMs_) >= timings_.heartbeatBlinkIntervalMs) {
    lastHeartbeatMs_ = nowMs;
    heartbeatOn_ = true;
    heartbeatOffMs_ = nowMs + timings_.heartbeatOnMs;
    indicator_.set(true);
  }

  if (heartbeatOn_ && nowMs >= heartbeatOffMs_) {
    heartbeatOn_ = false;
    indicator_.set(false);
  }
}

void App::maintainWifi(uint32_t nowMs) {
  if (wifi_.connected()) {
    return;
  }
  if (connectivityPolicy_.shouldAttempt(nowMs, wifi_.connected())) {
    connectivityPolicy_.markAttempt(nowMs);
    wifi_.connect(15000);
  }
}

void App::maybeSendTestPacket(uint32_t nowMs) {
  if (!enableTestPosts_) {
    return;
  }

  time_t now = time(nullptr);
  if (now < kValidTimeEpoch) {
    return;
  }
  uint32_t minuteEpoch = static_cast<uint32_t>((now / 60) * 60);
  if (minuteEpoch == lastTestMinute_) {
    return;
  }
  lastTestMinute_ = minuteEpoch;

  int k = static_cast<int>((++testCounter_) % 12);
  float windAvg = 10.0f + 0.5f * k;
  float windMax = windAvg * 1.5f;
  int windDir = 200 + 5 * k;

  char buf[128];
  std::snprintf(buf,
                sizeof(buf),
                "wind_avg=%.1f&wind_max=%.1f&wind_dir=%d&cnt=%lu",
                windAvg,
                windMax,
                windDir,
                static_cast<unsigned long>(testCounter_));
  std::string body(buf);

  core::Logger::info("TEST POSTING: %s", body.c_str());
  int code = http_.post(body);
  bool transient = (code < 0) || code == 408 || (code >= 500 && code < 600);
  if (transient) {
    core::Logger::warn("TEST transient (code=%d), retrying once...", code);
    clock_.delayMs(timings_.httpRetryDelayMs);
    code = http_.post(body);
  } else if (code >= 400 && code < 500 && code != 400) {
    core::Logger::warn("TEST client error (%d), skipping retry", code);
  }

  if (code == 200 || code == 201 || code == 400) {
    core::Logger::info("TEST POST accepted (%d)", code);
  } else {
    core::Logger::warn("TEST POST not successful (code=%d)", code);
  }
}

void App::handleParseFailure(domain::ParseStatus status) {
  switch (status) {
    case domain::ParseStatus::kEmpty:
      core::Logger::warn("BASE: empty payload, skip");
      break;
    case domain::ParseStatus::kMissingTrailer:
      core::Logger::warn("CRC trailer missing/short; dropping packet");
      break;
    case domain::ParseStatus::kTrailerNotHex:
      core::Logger::warn("CRC trailer not hex; dropping packet");
      break;
    case domain::ParseStatus::kCrcMismatch:
      core::Logger::warn("CRC mismatch; dropping packet");
      break;
    case domain::ParseStatus::kJsonError:
      core::Logger::warn("Parse fail: bad JSON, skipping packet");
      break;
    case domain::ParseStatus::kZeroPayload:
      core::Logger::warn("Packet looked valid but values are all zero; skipping");
      break;
    case domain::ParseStatus::kOk:
    default:
      break;
  }
}

void App::sendAck(int32_t counter, const std::string& status) {
  if (counter < 0) {
    return;
  }
  char head[64];
  std::snprintf(head, sizeof(head), "ACK %ld %s", static_cast<long>(counter), status.c_str());
  uint16_t crc = domain::LoRaPacket::ComputeCrc(head);
  char frame[80];
  std::snprintf(frame, sizeof(frame), "%s*%04X", head, static_cast<unsigned>(crc));
  core::Result tx = radio_.send(frame);
  if (!tx.ok) {
    core::Logger::error("BASE: ACK TX failed: %s", tx.message.c_str());
  } else {
    core::Logger::info("BASE: ACK TX (%s)", frame);
  }
}

int32_t App::computeDropGap(int32_t counter) {
  int32_t dropGap = 0;
  if (counter >= 0) {
    if (lastCntAccepted_ >= 0) {
      int32_t delta = counter - lastCntAccepted_;
      if (delta > 0) {
        if (nominalCntStep_ <= 0) {
          nominalCntStep_ = delta;
        } else {
          dropGap = (delta / nominalCntStep_) - 1;
          if (dropGap < 0) dropGap = 0;
          dropTotal_ += static_cast<uint32_t>(dropGap);
        }
      } else {
        nominalCntStep_ = -1;
      }
    }
    lastCntAccepted_ = counter;
  }
  return dropGap;
}

float App::computeDropRate(uint32_t deliveredNext) const {
  if ((dropTotal_ + deliveredNext) == 0) {
    return 0.0f;
  }
  return static_cast<float>(dropTotal_) /
         static_cast<float>(dropTotal_ + deliveredNext);
}

}  // namespace app
