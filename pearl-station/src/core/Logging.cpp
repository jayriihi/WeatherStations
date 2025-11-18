// Logging.cpp
// Implements the Logger helper, formatting log lines and forwarding them to
// whichever Arduino Print stream is registered during setup. Keeps the rest
// of the system from talking directly to Serial.

#include "core/Logging.h"

#include <Arduino.h>
#include <cstdarg>

namespace {
static Print* gLoggerOutput = nullptr;
}

namespace core {

void Logger::begin(Print* output) { gLoggerOutput = output; }

void Logger::info(const char* fmt, ...) {
  va_list args;
  va_start(args, fmt);
  vlog("[INFO]", fmt, args);
  va_end(args);
}

void Logger::warn(const char* fmt, ...) {
  va_list args;
  va_start(args, fmt);
  vlog("[WARN]", fmt, args);
  va_end(args);
}

void Logger::error(const char* fmt, ...) {
  va_list args;
  va_start(args, fmt);
  vlog("[ERR ]", fmt, args);
  va_end(args);
}

void Logger::vlog(const char* level, const char* fmt, va_list args) {
  if (!gLoggerOutput) {
    return;
  }

  static constexpr size_t kBufferSize = 256;
  char buffer[kBufferSize];
  vsnprintf(buffer, sizeof(buffer), fmt, args);

  gLoggerOutput->print(level);
  gLoggerOutput->print(' ');
  gLoggerOutput->println(buffer);
}

}  // namespace core
