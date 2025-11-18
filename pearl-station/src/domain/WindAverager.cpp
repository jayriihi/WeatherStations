// WindAverager.cpp
// Implements the pure-math accumulator that tracks speed sums, gust peaks,
// and direction vectors for each block. Produces WindSummary results for the
// App without relying on any hardware functions.

#include "domain/WindAverager.h"

#include <cmath>

namespace {
constexpr float kDegToRad = 0.01745329251994329577f;
constexpr float kRadToDeg = 57.295779513082320876f;
}

namespace domain {

void WindAverager::reset() {
  sampleCount_ = 0;
  sumSpeedMs_ = 0.0f;
  maxGustMs_ = 0.0f;
  sumDirX_ = 0.0f;
  sumDirY_ = 0.0f;
}

void WindAverager::addSample(const WindSample& sample) {
  if (!sample.valid) {
    return;
  }

  sumSpeedMs_ += sample.speedMs;
  float gust = (sample.gustMs > 0.0f) ? sample.gustMs : sample.speedMs;
  if (gust > maxGustMs_) {
    maxGustMs_ = gust;
  }

  float rad = sample.directionDeg * kDegToRad;
  sumDirX_ += std::cos(rad);
  sumDirY_ += std::sin(rad);
  sampleCount_++;
}

bool WindAverager::finalize(core::WindSummary& outSummary) {
  if (sampleCount_ == 0) {
    outSummary = core::WindSummary{};
    reset();
    return false;
  }

  outSummary.sampleCount = sampleCount_;
  outSummary.averageSpeedMs = sumSpeedMs_ / sampleCount_;
  outSummary.maxGustMs = maxGustMs_;

  float avgRad = std::atan2(sumDirY_, sumDirX_);
  float avgDeg = avgRad * kRadToDeg;
  if (avgDeg < 0.0f) {
    avgDeg += 360.0f;
  }
  outSummary.averageDirectionDeg = avgDeg;

  reset();
  return true;
}

}  // namespace domain
