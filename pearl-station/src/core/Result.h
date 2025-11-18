// Result.h
// Lightweight success/failure wrapper shared by drivers to report init errors
// without throwing exceptions. Holds a bool flag and a message for logging.

#pragma once

#include <string>
#include <utility>

namespace core {

struct Result {
  bool ok = true;
  std::string message;

  Result() = default;
  Result(bool success, std::string msg) : ok(success), message(std::move(msg)) {}

  static Result Success() { return Result(true, {}); }
  static Result Failure(std::string msg) { return Result(false, std::move(msg)); }
};

}  // namespace core
