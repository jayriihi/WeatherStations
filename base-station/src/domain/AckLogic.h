// AckLogic.h
// Encapsulates duplicate/in-flight detection and ACK status decisions for
// packets flowing through the base station.

#pragma once

#include <string>

namespace domain {

struct AckDecision {
  bool shouldPost = true;
  bool sendAck = false;
  std::string status;
};

class AckLogic {
 public:
  AckLogic(uint32_t inflightGuardMs, uint32_t minPostIntervalMs, uint32_t duplicateWindowMs);

  AckDecision evaluate(int32_t counter,
                       const std::string& body,
                       uint32_t nowMs) const;
  void markInflight(int32_t counter, uint32_t nowMs);
  void markSuccess(int32_t counter, const std::string& body, uint32_t nowMs);
  void expireInflight(uint32_t nowMs);

 private:
  uint32_t inflightGuardMs_;
  uint32_t minPostIntervalMs_;
  uint32_t duplicateWindowMs_;

  int32_t inflightCnt_ = -1;
  uint32_t inflightMs_ = 0;
  int32_t lastPostedCnt_ = -1;
  uint32_t lastPostMs_ = 0;
  std::string lastBody_;
  bool hasPosted_ = false;
};

}  // namespace domain
