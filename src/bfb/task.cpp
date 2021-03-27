#include "task.hpp"

namespace bfb {
Task::Task(std::uint32_t iPriority) : priority(iPriority) {
}

void Task::step() {
}

void Task::start() {
  task = std::make_unique<pros::Task>(
    [this]() {
      for (;;) {
        this->step();
      }
    },
    "BFB TASK");
}

void Task::stop() {
  task = nullptr;
}

} // namespace bfb