// Logging.cpp
// Implements the logger helper used throughout the base station firmware.
// Formats log strings and writes them through the registered Arduino Print.

#include "core/Logging.h"

#include <Arduino.h>

namespace {
Print* g_logOutput = nullptr;
}

namespace core {

void Logger::begin(Print* output) {
  g_logOutput = output;
}

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
  if (!g_logOutput) {
    return;
  }
  static constexpr size_t kBufferSize = 256;
  char buffer[kBufferSize];
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  g_logOutput->print(level);
  g_logOutput->print(' ');
  g_logOutput->println(buffer);
}

}  // namespace core
