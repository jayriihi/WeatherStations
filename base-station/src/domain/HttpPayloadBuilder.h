// HttpPayloadBuilder.h
// Assembles the application/x-www-form-urlencoded body for HTTP posts using
// parsed packet data plus health counters.

#pragma once

#include <string>

#include "core/Types.h"

namespace domain {

class HttpPayloadBuilder {
 public:
  std::string build(const core::HttpPostRequest& request) const;
};

}  // namespace domain
