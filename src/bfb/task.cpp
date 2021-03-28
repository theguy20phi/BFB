#include "task.hpp"

namespace bfb {
Task::Task(std::uint32_t iPriority) : priority(iPriority) {
}

void Task::task_fn() {
}

void Task::start() {
  task = std::make_unique<pros::Task>(
    [this]() {
      this->task_fn();
    },
    "BFB TASK");
}

void Task::stop() {
  task = nullptr;
}

} // namespace bfb