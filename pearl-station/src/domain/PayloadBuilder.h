// PayloadBuilder.h
// Declares the domain helper that assembles the JSON payload plus CRC trailer
// expected by the base station. Converts WindSummary data into the on-air
// format while remaining independent of hardware libraries.

#pragma once

#include <string>

#include "core/Types.h"

namespace domain {

class PayloadBuilder {
 public:
  std::string build(const core::WindSummary& summary,
                    float batteryVolts,
                    uint32_t counter) const;
};

}  // namespace domain
