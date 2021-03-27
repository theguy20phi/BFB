#pragma once

#include "pros/rtos.hpp"
#include "wait.hpp"
#include <memory>

namespace bfb {
class Task {
  public:
  Task(std::uint32_t iPriority = TASK_PRIORITY_DEFAULT);

  virtual void step();
  virtual void start() final;
  virtual void stop() final;

  protected:
  std::uint32_t priority;
  std::unique_ptr<pros::Task> task{nullptr};
};
} // namespace bfb