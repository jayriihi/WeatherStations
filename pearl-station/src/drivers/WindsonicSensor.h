// WindsonicSensor.h
// Declares the serial driver implementing IWindSensor for the Gill Windsonic.
// Handles UART setup, NMEA/Q sentence parsing, checksum checks, and optional
// simulation mode.

#pragma once

#include <HardwareSerial.h>

#include "core/Config.h"
#include "domain/WindSample.h"
#include "ports/IWindSensor.h"

namespace drivers {

class WindsonicSensor : public ports::IWindSensor {
 public:
  WindsonicSensor(HardwareSerial& serial, core::WindsonicSettings settings);
  void begin();
  bool readSample(domain::WindSample& sample) override;

 private:
  bool readLine(char* buf, size_t cap, uint32_t timeoutMs = 300);
  bool parseLine(char* line, domain::WindSample& sample);
  bool parseQLine(char* line, domain::WindSample& sample);
  bool parseMwvLine(char* line, domain::WindSample& sample);
  static bool nmeaChecksumValid(const char* sentence);
  static bool parseHexByte(char hi, char lo, uint8_t& out);
  static int hexDigitToInt(char c);
  float frand(float a, float b) const;
  void simulateSample(domain::WindSample& sample);

  HardwareSerial& serial_;
  core::WindsonicSettings settings_;
};

}  // namespace drivers
