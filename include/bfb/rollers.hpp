#pragma once

#include "api.h"
#include "task.hpp"
#include "values.hpp"
#include "wait.hpp"

namespace bfb {
enum class Direction {
  In,
  Out
};

class Rollers : public Task {
  public:
  Rollers();
  void task_fn() override;
  void standby();
  void intake();
  void outtake();
  void outtake_until_empty();
  void shoot();
  void shoot_until_empty();
  void shoot_and_outtake_until_empty();
  void shoot_and_outtake();
  void cycle();
  void intake_one_ball();
  void intake_two_balls();
  void toggle_shooter();
  void slow_rollers(bool slow);
  void coerce(const Direction &dir);
  void emergency_outtake();

  private:
  void command(double side_cmd, double l_shooter_cmd);
  bool is_in_lower();
  bool is_in_upper();

  private:
  bool intaking = false;
  const int threshold{2750};
  double power{600};
  bool shooting{true};
  pros::ADIAnalogIn upper_indexer{port::upper_line_tracker};
  pros::ADIAnalogIn lower_indexer{port::lower_line_tracker};
  pros::Motor lower_shooter{port::l_shooter_motor,
                            pros::motor_gearset_e_t::E_MOTOR_GEARSET_06,
                            false};
  pros::Motor upper_shooter{port::u_shooter_motor,
                            pros::motor_gearset_e_t::E_MOTOR_GEARSET_06,
                            false};
  pros::Motor left_side{port::l_side_motor, pros::motor_gearset_e_t::E_MOTOR_GEARSET_06, false};
  pros::Motor right_side{port::r_side_motor, pros::motor_gearset_e_t::E_MOTOR_GEARSET_06, true};
};
} // namespace bfb