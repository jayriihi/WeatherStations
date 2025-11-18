// WindSample.h
// Defines the raw sample struct shared between wind sensor drivers and the
// WindAverager domain logic. Keeps hardware parsing separate from statistics.

#pragma once

namespace domain {

struct WindSample {
  float speedMs = 0.0f;
  float gustMs = 0.0f;
  float directionDeg = 0.0f;
  bool valid = false;

  WindSample() = default;
  WindSample(float speed, float gust, float direction, bool v)
      : speedMs(speed), gustMs(gust), directionDeg(direction), valid(v) {}
};

}  // namespace domain
