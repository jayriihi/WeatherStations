// Scheduler.cpp
// Provides the concrete implementation of the simple periodic scheduler used
// by App. Tracks task intervals, executes callbacks when due, and lets the
// app delay tasks for jitter without touching hardware timers.

#include "app/Scheduler.h"

#include <utility>

namespace app {

Scheduler::TaskId Scheduler::schedule(uint32_t intervalMs, Callback cb) {
  Task task{intervalMs, intervalMs, std::move(cb), true};
  tasks_.push_back(std::move(task));
  return tasks_.size() - 1;
}

void Scheduler::tick(uint32_t nowMs) {
  for (auto& task : tasks_) {
    if (!task.active || !task.callback) {
      continue;
    }
    if (nowMs >= task.nextRunMs) {
      task.nextRunMs = nowMs + task.intervalMs;
      task.callback(nowMs);
    }
  }
}

void Scheduler::delayTask(TaskId id, uint32_t delayMs) {
  if (id >= tasks_.size()) {
    return;
  }
  tasks_[id].nextRunMs += delayMs;
}

}  // namespace app
