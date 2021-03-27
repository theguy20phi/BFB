#pragma once

#include "api.h"
#include "wait.hpp"

namespace bfb {
class Rollers {
  public:
  Rollers();
  void standby();
  void intake();
  void outtake();
  void shoot();
  void shoot_and_outtake();
  void cycle();
  void intake_one_ball();
  void intake_two_balls();
  void toggle_shooter();
  void slow_rollers(bool slow);

  private:
  void command(double side_cmd, double l_shooter_cmd);
  bool is_in_lower();
  bool is_in_upper();

  private:
  const int threshold{2750};
  double power{600};
  bool shooting{true};
  pros::ADIAnalogIn upper_indexer{'A'};
  pros::ADIAnalogIn lower_indexer{'B'};
  pros::Motor lower_shooter{10, pros::motor_gearset_e_t::E_MOTOR_GEARSET_06, false};
  pros::Motor upper_shooter{9, pros::motor_gearset_e_t::E_MOTOR_GEARSET_06, false};
  pros::Motor left_side{1, pros::motor_gearset_e_t::E_MOTOR_GEARSET_06, false};
  pros::Motor right_side{8, pros::motor_gearset_e_t::E_MOTOR_GEARSET_06, true};
};
} // namespace bfb