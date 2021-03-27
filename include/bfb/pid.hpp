#pragma once

#include "mathUtil.hpp"
#include "okapiUtil.hpp"
#include "wait.hpp"

namespace bfb {
class PID {
  public:
  struct Gains {
    double p = 0.0;
    double i = 0.0;
    double d = 0.0;
  };
  PID(const Gains &gains,
      SettledUtilPtr i_settled_util,
      const Range<double> &i_range = {-12000.0, 12000.0});
  void calculate(double state);
  double get_output() const;
  bool is_settled() const;
  void set_target(double state, double i_target);
  void reset();

  private:
  double calculate_i(double error);
  double calculate_d(double state);

  private:
  double output{0.0};
  double target{0.0};
  const Gains k;
  SettledUtilPtr settled_util;
  bool settled{true};
  const Range<double> range;
  int previous_time{0};
  double i{0.0};
  int previous_sign{1};
  static constexpr double I_DECAY{0.95};
  double previous_state{0.0};
};
} // namespace bfb