#include "chassis.hpp"

namespace bfb {
Chassis::Chassis() {
  set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  start();
}

void Chassis::task_fn() {
  for (pros::Imu imu : imus)
    imu.reset();
  bfb::wait(3000 * imus.size()); // Takes about three seconds for an IMU to calibrate.
  for (;;) {
    okapi::QAngle imu_reading{0.0_deg};
    for (pros::Imu imu : imus)
      imu_reading += imu.get_rotation() * (1.0 / imus.size()) * okapi::degree;
    okapi::QAngle delta_h_imu{imu_reading - previous_pose.h};
    const okapi::QLength l_delta{l_odom.distance_since()};
    const okapi::QLength r_delta{r_odom.distance_since()};
    const okapi::QLength s_delta{s_odom.distance_since()};
    okapi::QTime time{pros::millis() * okapi::millisecond};
    // TODO This might actually need to be okapi::degree
    const okapi::QAngle delta_h_odom{okapi::radian * (l_delta + r_delta) /
                                     (l_odom.distance_to_wheel() + r_odom.distance_to_wheel())};
    const okapi::QAngle delta_h{
      okapi::abs(delta_h_imu - delta_h_odom) > gyrodom_threshold ? delta_h_imu : delta_h_odom};
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
    pose = line_landmarker.correct_position(pose, line_sensor_triggered());
    pose = goal_landmarker.correct_position(pose, goal_limit_switch.get_value());
    previous_time = time;
    previous_pose = pose;
    wait(general_delay);
  }
}

void Chassis::drive_voltage(double forward, double strafe, double turn) {
  if (fabs(forward) + fabs(strafe) + fabs(turn) <= deadband)
    brake();
  else {
    double max_cmd{std::max(std::max(forward + strafe + turn, forward - strafe + turn),
                            std::max(forward - strafe - turn, forward + strafe - turn))};
    l_f_wheel.move_voltage((forward + strafe + turn) / max_cmd * 12000.0);
    l_b_wheel.move_voltage((forward - strafe + turn) / max_cmd * 12000.0);
    r_f_wheel.move_voltage((forward - strafe - turn) / max_cmd * 12000.0);
    r_b_wheel.move_voltage((forward + strafe - turn) / max_cmd * 12000.0);
  }
}

void Chassis::move_to(const std::vector<Pose> &targets,
                      bool involves_goal,
                      const okapi::QTime &timeout) {
  okapi::QTime time{pros::millis() * okapi::millisecond};
  for (Pose target : targets) {
    do {
      const okapi::QLength lateral_distance{-pose.distance_to(target)};
      const okapi::QAngle angular_distance(-okapi::OdomMath::constrainAngle180(target.h - pose.h));
      // TODO If something seems strange here, consider changing target.w to 0, for some reason...
      auto angular_target = angular_profiler.next_target(
        {angular_distance, pose.w}, {0.0_deg, target.w}, max_angular_vel, max_angular_accel);
      // TODO May need to change what max velocity equation is. Now may need to add it back.
      auto lateral_target = lateral_profiler.next_target(
        {lateral_distance, pose.v}, {0.0_in, target.v}, max_lateral_vel, max_lateral_accel);
      lateral_pos_pid.calculate(lateral_distance.convert(okapi::inch));
      angular_pos_pid.calculate(angular_distance.convert(okapi::radian));
      lateral_vel_pid.set_target(pose.v.convert(inps), lateral_target.v.convert(inps));
      lateral_vel_pid.calculate(pose.v.convert(inps));
      angular_vel_pid.set_target(pose.w.convert(okapi::radps),
                                 angular_target.v.convert(okapi::radps));
      angular_vel_pid.calculate(pose.w.convert(okapi::radps));
      const double lateral_command{lateral_pos_pid.get_output() + lateral_vel_pid.get_output() +
                                   lateral_kv * lateral_target.v.convert(inps) +
                                   lateral_ka * lateral_target.a.convert(inpsps)};
      const double angular_command(angular_pos_pid.get_output() + angular_vel_pid.get_output() +
                                   angular_kv * angular_target.v.convert(okapi::radps) +
                                   angular_ka * angular_target.a.convert(radpsps));
      drive_toward(target, lateral_command, angular_command);
      wait(general_delay);
    } while (
      !lateral_profiler.is_at_target({pose.distance_to(target), pose.v}, {0.0_in, target.v}) &&
      !angular_profiler.is_at_target(
        {okapi::OdomMath::constrainAngle180(target.h - pose.h), pose.w}, {0.0_deg, target.w}) &&
      !(involves_goal && goal_limit_switch.get_value()) &&
      !(timeout != 0.0_ms && pros::millis() * okapi::millisecond - time >= timeout));
  }
}

void Chassis::drive_toward(const Point &target, double command, double turn_command) {
  const Point coord_diff{target.x - pose.x, target.y - pose.y};
  // TODO May need to be negative pose.h
  const Point strafe_forward{coord_diff.rotate(pose.h)};
  const double strafe_command{
    (strafe_forward.x / okapi::abs(strafe_forward.x + strafe_forward.y)).convert(okapi::number)};
  const double forward_command{
    (strafe_forward.y / okapi::abs(strafe_forward.x + strafe_forward.y)).convert(okapi::number)};
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