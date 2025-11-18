// Scheduler.h
// Simple cooperative scheduler for running periodic callbacks in App.

#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

namespace app {

class Scheduler {
 public:
  using Callback = std::function<void(uint32_t)>;
  using TaskId = std::size_t;

  TaskId schedule(uint32_t intervalMs, Callback cb);
  void tick(uint32_t nowMs);
  void delayTask(TaskId id, uint32_t delayMs);

 private:
  struct Task {
    uint32_t intervalMs;
    uint32_t nextRunMs;
    Callback callback;
    bool active;
  };

  std::vector<Task> tasks_;
};

}  // namespace app
