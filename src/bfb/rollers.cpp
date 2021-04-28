#include "rollers.hpp"

namespace bfb {
Rollers::Rollers() {
  start();
  lower_shooter.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  upper_shooter.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  left_side.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  right_side.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}

void Rollers::task_fn() {
  for (;;) {
    if (intaking) {
      if (is_in_upper())
        command(600, 0);
      else
        command(600, 600);
    }
    wait(10);
  }
}

void Rollers::standby() {
  intaking = false;
  command(0, 0);
}

void Rollers::intake() {
  intaking = true;
  if (intaking) {
    if (is_in_upper())
      command(600, 0);
    else
      command(600, 600);
  }
}

void Rollers::outtake() {
  intaking = false;
  command(-600, -600);
}

void Rollers::outtake_until_empty() {
  intaking = false;
  outtake();
  wait_until([=]() { return !is_in_lower() && !is_in_upper(); }, 3000);
  wait(500);
}

void Rollers::shoot() {
  intaking = false;
  command(0, 600);
}

void Rollers::shoot_until_empty() {
  intaking = false;
  shoot();
  wait_until([=]() { return !is_in_lower() && !is_in_upper(); }, 3000);
  wait(750);
}

void Rollers::shoot_and_outtake_until_empty() {
  intaking = false;
  shoot_and_outtake();
  wait_until([=]() { return !is_in_lower() && !is_in_upper(); }, 3000);
  wait(750);
}

void Rollers::shoot_and_outtake() {
  intaking = false;
  command(-600, 600);
}

void Rollers::cycle() {
  intaking = false;
  command(600, 600);
}

void Rollers::intake_one_ball() {
  intaking = false;
  command(600, 0);
  wait_until([=]() { return is_in_lower(); }, 3000);
  standby();
}

void Rollers::intake_two_balls() {
  intaking = false;
  command(600, 600);
  wait_until([=]() { return is_in_upper(); }, 3000);
  intake_one_ball();
}

void Rollers::toggle_shooter() {
  shooting = !shooting;
}

void Rollers::slow_rollers(bool slow) {
  if (slow)
    power = 400;
  else
    power = 600;
}

void Rollers::coerce(const Direction &dir) {
  if(dir == Direction::In)
    command(100, 0);
  else
    command(-100, 0);
}

void Rollers::emergency_outtake() {
  left_side.move_velocity(-600);
  right_side.move_velocity(-600);
  lower_shooter.move_velocity(-600);
  upper_shooter.move_velocity(-600);
}

void Rollers::command(double side_cmd, double l_shooter_cmd) {
  left_side.move_velocity(side_cmd);
  right_side.move_velocity(side_cmd);
  lower_shooter.move_velocity(l_shooter_cmd);
  if (shooting)
    upper_shooter.move_velocity(power);
  else
    upper_shooter.move_velocity(0);
}

bool Rollers::is_in_lower() {
  return lower_indexer.get_value() < threshold;
}

bool Rollers::is_in_upper() {
  return upper_indexer.get_value() < threshold;
}
} // namespace bfb