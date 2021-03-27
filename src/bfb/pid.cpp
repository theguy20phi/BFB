#include "pid.hpp"

namespace bfb {
PID::PID(const Gains &gains, SettledUtilPtr i_settled_util, const Range<double> &i_range)
  : k(gains), settled_util(std::move(i_settled_util)), range(i_range) {
}

void PID::calculate(double state) {
  uint32_t current_time{pros::millis()};
  if (current_time - previous_time < general_delay)
    return;
  previous_time = current_time;
  const double error{target - state};
  settled = settled_util->isSettled(error);
  output = clamp(k.p * error + calculate_i(error) + calculate_d(state), range);
}

double PID::calculate_i(double error) {
  i *= I_DECAY;
  i += error * k.i;
  int current_sign{sign(error)};
  if (current_sign != previous_sign)
    i = 0;
  previous_sign = current_sign;
  return i;
}

double PID::calculate_d(double state) {
  const double d{previous_state - state};
  previous_state = state;
  return d * k.d;
}

double PID::get_output() const {
  return output;
}

bool PID::is_settled() const {
  return settled;
}

void PID::set_target(double state, double i_target) {
  target = i_target;
  previous_state = state;
  i = 0.0;
}

void PID::reset() {
  target = 0.0;
  output = 0.0;
  previous_state = 0.0;
  i = 0.0;
  previous_sign = 1;
  previous_time = pros::millis();
  settled_util->reset();
  settled = true;
}
} // namespace bfb