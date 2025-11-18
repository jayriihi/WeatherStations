// Result.h
// Lightweight success/failure helper so drivers can report initialization or
// runtime errors without exceptions. Used across app/domain layers.

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
