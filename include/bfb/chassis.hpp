#pragma once

#include "api.h"
#include "motionProfiler.hpp"
#include "okapi/api.hpp"
#include "task.hpp"

using namespace okapi::literals;

namespace bfb {
struct Point {
  okapi::QLength x{0.0};
  okapi::QLength y{0.0};
  okapi::QAngle h{0.0};
  okapi::QLength distance_to(const Point &other) const;
  Point rotate(const okapi::QAngle &ccw) const;
};

struct Pose : public Point {
  okapi::QSpeed v{0.0};
  okapi::QAngularSpeed w{0.0};
};

class Chassis : public Task {
  public:
  Chassis();
  void step() override;
  void drive_voltage(double forward, double strafe, double turn);
  void move_to(const std::vector<Pose> &targets, bool involves_goal = false);
  void drive_toward(const Point &target);
  void brake();
  void set_brake_mode(const pros::motor_brake_mode_e_t &brake_mode);
  void toggle_hold();
  void set_pose(const Pose &i_pose);
  Pose get_pose() const;
};
} // namespace bfb