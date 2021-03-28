#pragma once

#include "api.h"
#include "landmarker.hpp"
#include "motionProfiler.hpp"
#include "odometer.hpp"
#include "pid.hpp"
#include "task.hpp"
#include "values.hpp"

using namespace okapi::literals;

namespace bfb {
class Chassis : public Task {
  public:
  Chassis();
  void step() override;
  void drive_voltage(double forward, double strafe, double turn);
  void move_to(Pose &target, bool involves_goal = false);
  void drive_toward(const Point &target);
  void brake();
  void set_brake_mode(const pros::motor_brake_mode_e_t &brake_mode);
  void toggle_hold();
  void set_pose(const Pose &i_pose);
  Pose get_pose() const;

  private:
  pros::Motor l_f_wheel{port::l_f_drive_motor};
  pros::Motor l_b_wheel{port::l_b_drive_motor};
  pros::Motor r_f_wheel{port::r_f_drive_motor, true};
  pros::Motor r_b_wheel{port::r_b_drive_motor, true};
  const double deadband{250.0};
  // NOTE Tune these!
  PID lateral_pos_pid{{0, 0, 0}, make_settled_util()};
  PID lateral_vel_pid{{0, 0, 0}, make_settled_util()};
  PID heading_pos_pid{{0, 0, 0}, make_settled_util()};
  PID heading_vel_pid{{0, 0, 0}, make_settled_util()};
  MotionProfiler<okapi::QLength, okapi::QSpeed, okapi::QAcceleration> lateral_profiler{0.5_in};
  MotionProfiler<okapi::QAngle, okapi::QAngularSpeed, okapi::QAngularAcceleration> heading_profiler{
    1.0_deg};

  Pose pose{};
  // NOTE Get real values here! (also make sure top port is the char and bot port is the next char)
  Odometer l_odom{port::l_encoder, 8.0_in};
  Odometer r_odom{port::r_encoder, 8.0_in};
  Odometer s_odom{port::s_encoder, 8.0_in};
  pros::ADILineSensor center_line_sensor{{port::port_extender, port::center_line_tracker}};
  // NOTE May be normally closed instead of normally open
  pros::ADIDigitalIn goal_limit_switch{{port::port_extender, port::goal_limit_switch}};
};
} // namespace bfb