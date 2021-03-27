#include "wait.hpp"

namespace bfb {
void wait_until(const std::function<bool()> &condition, const int max_delay) {
  std::uint32_t timeout{pros::millis() + max_delay};
  wait_until([condition, timeout]() { return condition() || pros::millis() >= timeout; });
}

void wait_until(const std::function<bool()> &condition) {
  while (!condition())
    wait(general_delay);
}

void wait(const int ms) {
  std::uint32_t now{pros::millis()};
  pros::Task::delay_until(&now, ms);
}
} // namespace bfb