// LedIndicator.cpp
// GPIO-backed LED indicator for heartbeat/status signaling.

#include "drivers/LedIndicator.h"

#include <Arduino.h>

namespace drivers {

LedIndicator::LedIndicator(int pin) : pin_(pin) {}

void LedIndicator::begin() {
  pinMode(pin_, OUTPUT);
  digitalWrite(pin_, LOW);
}

void LedIndicator::set(bool on) {
  digitalWrite(pin_, on ? HIGH : LOW);
}

}  // namespace drivers
