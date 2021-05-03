#pragma once

#include "api.h"
#include "landmarker.hpp"
#include "motionProfiler.hpp"
#include "odometer.hpp"
#include "okapiUtil.hpp"
#include "pid.hpp"
#include "task.hpp"
#include "values.hpp"
#include <algorithm>

using namespace okapi::literals;
using namespace bfb::literals;

namespace bfb {
class Chassis : public Task {
  public:
  Chassis();
  void task_fn() override;
  void drive_voltage(double forward, double strafe, double turn);
  void move_to(const std::vector<Pose> &targets,
               bool involves_goal = false,
               const okapi::QTime &timeout = 9000.0_s);
  void drive_toward(const Point &target, double command, double turn_command);
  void brake();
  void set_brake_mode(const pros::motor_brake_mode_e_t &brake_mode);
  void toggle_hold();
  void set_pose(const Pose &i_pose);
  Pose get_pose() const;
  void reset();

  private:
  bool line_sensor_triggered() const;

  private:
  pros::Motor l_f_wheel{port::l_f_drive_motor};
  pros::Motor l_b_wheel{port::l_b_drive_motor};
  pros::Motor r_f_wheel{port::r_f_drive_motor, true};
  pros::Motor r_b_wheel{port::r_b_drive_motor, true};
  const double deadband{2000.0};
  PID lateral_pos_pid{{2750.0, 0.0, 42500}, make_settled_util(1.5, 0.5)};
  PID angular_pos_pid{{1500.0, 0.0, 3250}, make_settled_util(1.5, 0.5)};
  Pose pose{};
  Pose previous_pose{};
  okapi::QTime previous_time{0_ms};
  Odometer l_odom{port::l_encoder, 7.0703125_in, true};
  Odometer r_odom{port::r_encoder, 7.0703125_in, true};
  Odometer s_odom{port::s_encoder, 5.5_in, true};
  pros::ADILineSensor center_line_sensor{{port::port_extender, port::center_line_tracker}};
  static constexpr int line_threshold{500};
  pros::ADIDigitalIn goal_limit_switch{{port::port_extender, port::goal_limit_switch}};
  LineLandmarker line_landmarker{
    {{{0.0_tile, 1.0_tile - 0.625_in}, {6.0_tile, 1.0_tile - 0.625_in}},
     {{0.0_tile, 3.0_tile - 0.625_in}, {6.0_tile, 3.0_tile - 0.625_in}},
     {{0.0_tile, 3.0_tile + 0.625_in}, {6.0_tile, 3.0_tile + 0.625_in}},
     {{0.0_tile, 5.0_tile + 0.625_in}, {6.0_tile, 5.0_tile + 0.625_in}}},
    {{{2.5_tile, 1.5_tile}, {3.5_tile, 0.0_tile}}, {{2.5_tile, 4.5_tile}, {3.5_tile, 6.0_tile}}},
    0.625};
  static constexpr okapi::QLength goal_radius{14.5_in};
  GoalLandmarker goal_landmarker{{{{3.0_tile, 3.0_tile}, goal_radius},
                                  {{5.65_in, 5.65_in}, goal_radius},
                                  {{3.0_tile, 5.65_in}, goal_radius},
                                  {{6.0_tile - 5.65_in, 5.65_in}, goal_radius},
                                  {{5.65_in, 3.0_tile}, goal_radius},
                                  {{6.0_tile - 5.65_in, 3.0_tile}, goal_radius},
                                  {{5.65_in, 6.0_tile - 5.65_in}, goal_radius},
                                  {{3.0_tile, 6.0_tile - 5.65_in}, goal_radius},
                                  {{6.0_tile - 5.65_in, 6.0_tile - 5.65_in}, goal_radius}},
                                 0.675};
};
} // namespace bfb