// WindsonicSensor.cpp
// Implements the UART reader for the Gill Windsonic sensor. Reads lines,
// validates checksums, parses Q/MWV formats, and supplies WindSample data via
// the IWindSensor interface or a simulator when disabled.

#include "drivers/WindsonicSensor.h"

#include <Arduino.h>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <esp_system.h>

namespace drivers {

WindsonicSensor::WindsonicSensor(HardwareSerial& serial,
                                 core::WindsonicSettings settings)
    : serial_(serial), settings_(settings) {}

void WindsonicSensor::begin() {
  if (!settings_.enabled) {
    return;
  }
  serial_.begin(settings_.baud, SERIAL_8N1, settings_.rxPin, settings_.txPin);
}

bool WindsonicSensor::readSample(domain::WindSample& sample) {
  if (!settings_.enabled) {
    simulateSample(sample);
    return true;
  }

  char line[96];
  if (!readLine(line, sizeof(line))) {
    sample.valid = false;
    return false;
  }

  if (!parseLine(line, sample)) {
    sample.valid = false;
    return false;
  }

  sample.valid = true;
  return true;
}

bool WindsonicSensor::readLine(char* buf, size_t cap, uint32_t timeoutMs) {
  uint32_t start = millis();
  size_t n = 0;
  while (millis() - start < timeoutMs) {
    while (serial_.available()) {
      char c = static_cast<char>(serial_.read());
      if (c == '\r') {
        continue;
      }
      if (c == '\n') {
        buf[n] = '\0';
        return n > 0;
      }
      if (n + 1 < cap) {
        buf[n++] = c;
      }
    }
  }
  return false;
}

bool WindsonicSensor::parseLine(char* line, domain::WindSample& sample) {
  if (line[0] == '$') {
    if (!nmeaChecksumValid(line)) {
      return false;
    }
    char* star = std::strchr(line, '*');
    if (star) {
      *star = '\0';
    }
  }

  if (parseQLine(line, sample)) {
    return true;
  }

  if (parseMwvLine(line, sample)) {
    return true;
  }

  return false;
}

bool WindsonicSensor::parseQLine(char* line, domain::WindSample& sample) {
  if (!line) {
    return false;
  }
  if ((line[0] != 'Q' && line[0] != 'q') || line[1] != ',') {
    return false;
  }

  char* p = line + 2;
  char* t1 = std::strtok(p, ",");
  char* t2 = std::strtok(nullptr, ",");
  char* t3 = std::strtok(nullptr, ",");
  if (!t1 || !t2) {
    return false;
  }

  float dir = std::atof(t1);
  float spd = std::atof(t2);
  float gust = t3 ? std::atof(t3) : spd;

  if (dir < 0.0f || dir > 360.0f || spd < 0.0f) {
    return false;
  }

  sample.directionDeg = dir;
  sample.speedMs = spd;
  sample.gustMs = gust;
  return true;
}

bool WindsonicSensor::parseMwvLine(char* line, domain::WindSample& sample) {
  if (!line) {
    return false;
  }

  if (std::strstr(line, "MWV") == nullptr) {
    return false;
  }

  char* p = (line[0] == '$') ? line + 1 : line;
  char* comma = std::strchr(p, ',');
  if (!comma) {
    return false;
  }
  p = comma + 1;

  char* angle = std::strtok(p, ",");
  std::strtok(nullptr, ",");  // skip R/T
  char* spd = std::strtok(nullptr, ",");
  char* unit = std::strtok(nullptr, ",");
  if (!angle || !spd || !unit) {
    return false;
  }

  float dir = std::atof(angle);
  float v = std::atof(spd);
  float spdMs = (*unit == 'N') ? v * 0.5144444f
                : (*unit == 'K') ? v / 3.6f
                                  : v;

  if (dir < 0.0f || dir > 360.0f || spdMs < 0.0f) {
    return false;
  }

  sample.directionDeg = dir;
  sample.speedMs = spdMs;
  sample.gustMs = spdMs;
  return true;
}

int WindsonicSensor::hexDigitToInt(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
  if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
  return -1;
}

bool WindsonicSensor::parseHexByte(char hi, char lo, uint8_t& out) {
  int hiVal = hexDigitToInt(hi);
  int loVal = hexDigitToInt(lo);
  if (hiVal < 0 || loVal < 0) {
    return false;
  }
  out = static_cast<uint8_t>((hiVal << 4) | loVal);
  return true;
}

bool WindsonicSensor::nmeaChecksumValid(const char* sentence) {
  if (!sentence) {
    return false;
  }

  const char* p = std::strchr(sentence, '$');
  if (!p) {
    return false;
  }
  p++;

  const char* star = std::strchr(p, '*');
  if (!star || !star[1] || !star[2]) {
    return false;
  }

  uint8_t checksum = 0;
  const char* q = p;
  while (q < star) {
    checksum ^= static_cast<uint8_t>(*q);
    ++q;
  }

  uint8_t msgChecksum = 0;
  if (!parseHexByte(star[1], star[2], msgChecksum)) {
    return false;
  }

  return checksum == msgChecksum;
}

float WindsonicSensor::frand(float a, float b) const {
  uint32_t rnd = esp_random();
  float f = static_cast<float>(rnd) / static_cast<float>(UINT32_MAX);
  return a + (b - a) * f;
}

void WindsonicSensor::simulateSample(domain::WindSample& sample) {
  sample.speedMs = frand(8.0f, 12.0f);
  if (frand(0.0f, 1.0f) > 0.97f) {
    sample.speedMs = frand(13.0f, 17.0f);
  }
  sample.gustMs = sample.speedMs;
  sample.directionDeg = 70.0f + frand(-8.0f, 8.0f);
  if (sample.directionDeg < 0.0f) sample.directionDeg += 360.0f;
  if (sample.directionDeg >= 360.0f) sample.directionDeg -= 360.0f;
  sample.valid = true;
}

}  // namespace drivers
