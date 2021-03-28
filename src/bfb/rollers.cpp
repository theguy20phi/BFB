#include "rollers.hpp"

namespace bfb {
Rollers::Rollers() {
  lower_shooter.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  upper_shooter.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  left_side.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  right_side.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}

void Rollers::standby() {
  command(0, 0);
}

void Rollers::intake() {
  if (is_in_upper())
    command(600, 0);
  else
    command(600, 600);
}

void Rollers::outtake() {
    command(-600, -600);
}

void Rollers::outtake_until_empty() {
  outtake();
  wait_until([=]() { return !is_in_lower() && !is_in_upper(); }, 3000);
  wait(500);
}

void Rollers::shoot() {
    command(0, 600);
}

void Rollers::shoot_until_empty() {
  shoot();
  wait_until([=]() { return !is_in_lower() && !is_in_upper(); }, 3000);
  wait(500);
}

void Rollers::shoot_and_outtake() {
    command(-600, 600);
}

void Rollers::cycle() {
    command(600, 600);
}

void Rollers::intake_one_ball() {
    command(600, 0);
    wait_until([=](){ return is_in_lower(); }, 3000);
    standby();
}

void Rollers::intake_two_balls() {
    command(600, 600);
    wait_until([=](){ return is_in_upper(); }, 3000);
    intake_one_ball();
}

void Rollers::toggle_shooter() {
    shooting = !shooting;
}

void Rollers::slow_rollers(bool slow) {
    if(slow)
        power = 400;
    else
        power = 600;
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