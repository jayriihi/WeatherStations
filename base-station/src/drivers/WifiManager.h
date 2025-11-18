// WifiManager.h
// Implements the IWifi interface by cycling through configured SSIDs.

#pragma once

#include <cstddef>

#include "core/Config.h"
#include "ports/IWifi.h"

namespace drivers {

class WifiManager : public ports::IWifi {
 public:
  WifiManager(const core::WifiCredential* creds, std::size_t count);

  void connect(uint32_t maxWaitMs) override;
  bool connected() const override;

 private:
  const core::WifiCredential* creds_;
  std::size_t count_;
};

}  // namespace drivers
