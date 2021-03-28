#pragma once

#include "api.h"
#include "landmarker.hpp"
#include "motionProfiler.hpp"
#include "odometer.hpp"
#include "pid.hpp"
#include "task.hpp"

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
  pros::Motor lFWheel{11};
  pros::Motor lBWheel{20};
  pros::Motor rFWheel{12, true};
  pros::Motor rBWheel{18, true};
  const double deadband{250.0};
  // NOTE Tune these!
  PID lateral_pos_pid{{0, 0, 0}, make_settled_util()};
  PID lateral_vel_pid{{0, 0, 0}, make_settled_util()};
  PID heading_pos_pid{{0, 0, 0}, make_settled_util()};
  PID heading_vel_pid{{0, 0, 0}, make_settled_util()};
  MotionProfiler<okapi::QLength, okapi::QSpeed, okapi::QAcceleration> lateral_profiler{0.5_in};
  MotionProfiler<okapi::QAngle, okapi::QAngularSpeed, okapi::QAngularAcceleration> heading_profiler{
    1.0_deg};
};
} // namespace bfb