// WindAverager.h
// Domain component responsible for accumulating per-sample wind data over a
// block and producing averaged speed, gust, and circular direction summaries
// without touching any Arduino-specific APIs.

#pragma once

#include "core/Types.h"
#include "domain/WindSample.h"

namespace domain {

class WindAverager {
 public:
  void reset();
  void addSample(const WindSample& sample);
  bool finalize(core::WindSummary& outSummary);
  uint16_t sampleCount() const { return sampleCount_; }

 private:
  uint16_t sampleCount_ = 0;
  float sumSpeedMs_ = 0.0f;
  float maxGustMs_ = 0.0f;
  float sumDirX_ = 0.0f;
  float sumDirY_ = 0.0f;
};

}  // namespace domain
