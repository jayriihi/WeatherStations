// Logging.h
// Central logger forwarding printf-style messages to an Arduino Print target.
// Allows app/domain code to log without depending directly on Serial.

#pragma once

#include <cstdarg>

class Print;

namespace core {

class Logger {
 public:
  static void begin(Print* output);
  static void info(const char* fmt, ...);
  static void warn(const char* fmt, ...);
  static void error(const char* fmt, ...);

 private:
  static void vlog(const char* level, const char* fmt, va_list args);
};

}  // namespace core
