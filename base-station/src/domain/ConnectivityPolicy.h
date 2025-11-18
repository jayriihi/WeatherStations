// ConnectivityPolicy.h
// Optional helper that determines when WiFi reconnect attempts should run,
// preventing constant retries when already connected.

#pragma once

namespace domain {

class ConnectivityPolicy {
 public:
  explicit ConnectivityPolicy(uint32_t retryIntervalMs)
      : retryIntervalMs_(retryIntervalMs) {}

  bool shouldAttempt(uint32_t nowMs, bool currentlyConnected) const {
    if (currentlyConnected) {
      return false;
    }
    return (nowMs - lastAttemptMs_) >= retryIntervalMs_;
  }

  void markAttempt(uint32_t nowMs) { lastAttemptMs_ = nowMs; }

 private:
  uint32_t retryIntervalMs_;
  uint32_t lastAttemptMs_ = 0;
};

}  // namespace domain
