// IWindSensor.h
// Interface for retrieving a wind sample. Allows app/domain code to depend on
// abstract data rather than UART parsing, implemented by WindsonicSensor.

#pragma once

#include "domain/WindSample.h"

namespace ports {

class IWindSensor {
 public:
  virtual ~IWindSensor() = default;
  virtual bool readSample(domain::WindSample& sample) = 0;
};

}  // namespace ports
