// Types.h
// Shared type definitions for common data flowing between layers. Currently
// holds the WindSummary struct and constants like the m/s to knots factor
// used by domain and app code.

#pragma once

#include <cstdint>

namespace core {

struct WindSummary {
  float averageSpeedMs = 0.0f;
  float maxGustMs = 0.0f;
  float averageDirectionDeg = 0.0f;
  uint16_t sampleCount = 0;
};

constexpr float kMsToKnots = 1.94384f;

}  // namespace core
