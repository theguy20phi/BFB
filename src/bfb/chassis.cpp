#include "chassis.hpp"

namespace bfb {
Chassis::Chassis() {
  set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  start();
}

void Chassis::task_fn() {
  for (;;) {
    const okapi::QLength l_delta{l_odom.distance_since()};
    const okapi::QLength r_delta{r_odom.distance_since()};
    const okapi::QLength s_delta{s_odom.distance_since()};
    okapi::QTime time{pros::millis() * okapi::millisecond};
    const okapi::QAngle delta_h{
      okapi::radian *
      ((l_delta - r_delta) / (l_odom.distance_to_wheel() + r_odom.distance_to_wheel()))};
    okapi::QLength local_x{0.0_in};
    okapi::QLength local_y{0.0_in};
    if (delta_h != 0.0_deg) {
      const auto two_sin_half_delta_h = 2.0 * okapi::sin(delta_h / 2.0);
      local_x = two_sin_half_delta_h *
                (s_delta / delta_h.convert(okapi::radian) + s_odom.distance_to_wheel());
      local_y = two_sin_half_delta_h *
                (r_delta / delta_h.convert(okapi::radian) + r_odom.distance_to_wheel());
    } else {
      local_x = s_delta;
      local_y = r_delta;
    }
    const okapi::QLength magnitude{okapi::sqrt(local_x * local_x + local_y * local_y)};
    const okapi::QAngle direction{okapi::atan2(local_y, local_x) -
                                  (previous_pose.h + delta_h / 2.0)};
    const okapi::QLength delta_x{okapi::cos(direction) * magnitude};
    const okapi::QLength delta_y{okapi::sin(direction) * magnitude};
    pose.x += delta_x;
    pose.y += delta_y;
    pose.h += delta_h;
    const okapi::QTime delta_t{time - previous_time};
    pose.v = previous_pose.distance_to(pose) / delta_t;
    pose.w = (pose.h - previous_pose.h) / delta_t;
    previous_time = time;
    previous_pose = pose;
    wait(general_delay);
  }
}

void Chassis::drive_voltage(double forward, double strafe, double turn) {
  if (fabs(forward) + fabs(strafe) + fabs(turn) <= deadband)
    brake();
  else {
    double max_cmd{
      std::max(std::max(fabs(forward + strafe + turn), fabs(forward - strafe + turn)),
               std::max(fabs(forward - strafe - turn), fabs(forward + strafe - turn)))};
    l_f_wheel.move_voltage((forward + strafe + turn) / max_cmd * std::min(12000.0, max_cmd));
    l_b_wheel.move_voltage((forward - strafe + turn) / max_cmd * std::min(12000.0, max_cmd));
    r_f_wheel.move_voltage((forward - strafe - turn) / max_cmd * std::min(12000.0, max_cmd));
    r_b_wheel.move_voltage((forward + strafe - turn) / max_cmd * std::min(12000.0, max_cmd));
  }
}

void Chassis::move_to(const std::vector<Pose> &targets,
                      bool involves_goal,
                      const okapi::QTime &timeout) {
  for (Pose target : targets) {
    okapi::QTime time{pros::millis() * okapi::millisecond};
    do {
      const okapi::QLength lateral_distance{-pose.distance_to(target)};
      const okapi::QAngle angular_distance(-okapi::OdomMath::constrainAngle180(target.h - pose.h));
      lateral_pos_pid.calculate(lateral_distance.convert(okapi::inch));
      angular_pos_pid.calculate(angular_distance.convert(okapi::degree));
      const double lateral_command{lateral_pos_pid.get_output()};
      const double angular_command(angular_pos_pid.get_output());
      drive_toward(target, lateral_command, angular_command);
      wait(general_delay);
    } while ((!lateral_pos_pid.is_settled() || !angular_pos_pid.is_settled()) &&
             !(involves_goal && goal_limit_switch.get_value()) &&
             !(pros::millis() * okapi::millisecond - time >= timeout));
  }
  brake();
}

void Chassis::drive_toward(const Point &target, double command, double turn_command) {
  const Point coord_diff{target.x - pose.x, target.y - pose.y};
  const okapi::QAngle angle_to{okapi::atan2(coord_diff.y, coord_diff.x) + pose.h};
  const double strafe_command{command * okapi::cos(angle_to).convert(okapi::number)};
  const double forward_command{command * okapi::sin(angle_to).convert(okapi::number)};
  drive_voltage(forward_command, strafe_command, turn_command);
}

void Chassis::brake() {
  l_f_wheel.move_velocity(0);
  l_b_wheel.move_velocity(0);
  r_f_wheel.move_velocity(0);
  r_b_wheel.move_velocity(0);
}

void Chassis::set_brake_mode(const pros::motor_brake_mode_e_t &brake_mode) {
  l_f_wheel.set_brake_mode(brake_mode);
  l_b_wheel.set_brake_mode(brake_mode);
  r_f_wheel.set_brake_mode(brake_mode);
  r_b_wheel.set_brake_mode(brake_mode);
}

void Chassis::toggle_hold() {
  if (l_f_wheel.get_brake_mode() == pros::E_MOTOR_BRAKE_COAST)
    set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  else
    set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}

void Chassis::set_pose(const Pose &i_pose) {
  pose = i_pose;
  previous_pose = pose;
}

Pose Chassis::get_pose() const {
  return pose;
}

void Chassis::reset() {
  set_pose({0.0_in, 0.0_in, 0.0_deg});
  brake();
}

bool Chassis::line_sensor_triggered() const {
  return center_line_sensor.get_value() < line_threshold;
}
} // namespace bfb