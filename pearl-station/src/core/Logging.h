// Logging.h
// Central logger that routes formatted printf-style messages to an Arduino
// Print sink. Used by app and drivers to keep logging consistent and
// hardware-agnostic outside of this helper.

#pragma once

#include <cstdarg>

class Print;  // forward declaration from Arduino core

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
