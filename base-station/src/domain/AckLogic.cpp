// AckLogic.cpp
// Implements duplicate/rate-limit detection and ACK status selection.

#include "domain/AckLogic.h"

namespace domain {

AckLogic::AckLogic(uint32_t inflightGuardMs,
                   uint32_t minPostIntervalMs,
                   uint32_t duplicateWindowMs)
    : inflightGuardMs_(inflightGuardMs),
      minPostIntervalMs_(minPostIntervalMs),
      duplicateWindowMs_(duplicateWindowMs) {}

AckDecision AckLogic::evaluate(int32_t counter,
                               const std::string& body,
                               uint32_t nowMs) const {
  AckDecision decision;
  decision.shouldPost = true;

  if (counter >= 0 && inflightCnt_ >= 0 && counter == inflightCnt_) {
    if ((nowMs - inflightMs_) < inflightGuardMs_) {
      decision.shouldPost = false;
      decision.sendAck = true;
      decision.status = "DUP";
      return decision;
    }
  }

  if (hasPosted_ && counter >= 0 && counter == lastPostedCnt_) {
    decision.shouldPost = false;
    decision.sendAck = true;
    decision.status = "DUP";
    return decision;
  }

  if (hasPosted_ && lastPostMs_ != 0 && (nowMs - lastPostMs_) < minPostIntervalMs_) {
    decision.shouldPost = false;
    if (counter >= 0) {
      decision.sendAck = true;
      decision.status = "RATE";
    }
    return decision;
  }

  if (hasPosted_ && !body.empty() && !lastBody_.empty() && lastPostMs_ != 0 &&
      body == lastBody_ && (nowMs - lastPostMs_) < duplicateWindowMs_) {
    decision.shouldPost = false;
    if (counter >= 0) {
      decision.sendAck = true;
      decision.status = "DUP";
    }
    return decision;
  }

  decision.shouldPost = true;
  decision.sendAck = false;
  decision.status.clear();
  return decision;
}

void AckLogic::markInflight(int32_t counter, uint32_t nowMs) {
  inflightCnt_ = counter;
  inflightMs_ = nowMs;
}

void AckLogic::markSuccess(int32_t counter,
                           const std::string& body,
                           uint32_t nowMs) {
  lastPostedCnt_ = counter;
  lastPostMs_ = nowMs;
  lastBody_ = body;
  inflightCnt_ = -1;
  hasPosted_ = true;
}

void AckLogic::expireInflight(uint32_t nowMs) {
  if (inflightCnt_ >= 0 && (nowMs - inflightMs_) >= inflightGuardMs_) {
    inflightCnt_ = -1;
  }
}

}  // namespace domain
