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
               const okapi::QTime &timeout = 0.0_ms);
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
  const double deadband{500.0};
  // TODO Tune these!
  PID lateral_pos_pid{{0, 0, 0}, make_settled_util()};
  PID lateral_vel_pid{{0, 0, 0}, make_settled_util()};
  PID angular_pos_pid{{0, 0, 0}, make_settled_util()};
  PID angular_vel_pid{{0, 0, 0}, make_settled_util()};
  MotionProfiler<okapi::QLength, okapi::QSpeed, okapi::QAcceleration> lateral_profiler{0.5_in};
  MotionProfiler<okapi::QAngle, okapi::QAngularSpeed, okapi::QAngularAcceleration> angular_profiler{
    1.0_deg};
  // TODO Have actual values here!
  static constexpr okapi::QSpeed max_lateral_vel{0.1_inps};
  static constexpr okapi::QAngularSpeed max_angular_vel{0.1_radps};
  static constexpr okapi::QAcceleration max_lateral_accel{0.01_inpsps};
  static constexpr okapi::QAngularAcceleration max_angular_accel{0.01_radpsps};
  // TODO 16 comes from the track of the robot in inches.
  static constexpr double curvature_constraint{16.0};
  // TODO Tune these! Start with kv = 1 / max_vel.
  static constexpr double lateral_kv{0.0};
  static constexpr double lateral_ka{0.0};
  static constexpr double angular_kv{0.0};
  static constexpr double angular_ka{0.0};
  Pose pose{};
  Pose previous_pose{};
  okapi::QTime previous_time{0_ms};
  // TODO This might actually be _deg
  const okapi::QAngle gyrodom_threshold{0.075_rad};
  std::vector<pros::Imu> imus{
    {pros::Imu{port::imu_a}, pros::Imu{port::imu_b}, pros::Imu{port::imu_c}}};
  // TODO Get real values here! (also make sure top port is the char and bot port is the next char)
  Odometer l_odom{port::l_encoder, 8.0_in};
  Odometer r_odom{port::r_encoder, 8.0_in};
  Odometer s_odom{port::s_encoder, 8.0_in};
  pros::ADILineSensor center_line_sensor{{port::port_extender, port::center_line_tracker}};
  // TODO The threshold at which the line sensor triggers may be different.
  static constexpr int line_threshold{500};
  // TODO May be normally closed instead of normally open
  pros::ADIDigitalIn goal_limit_switch{{port::port_extender, port::goal_limit_switch}};
  LineLandmarker line_landmarker{
    {{{0.0_tile, 1.0_tile - 1.0_in}, {6.0_tile, 1.0_tile - 1.0_in}},
     {{0.0_tile, 3.0_tile - 1.0_in}, {6.0_tile, 3.0_tile - 1.0_in}},
     {{0.0_tile, 3.0_tile + 1.0_in}, {6.0_tile, 3.0_tile + 1.0_in}},
     {{0.0_tile, 5.0_tile + 1.0_in}, {6.0_tile, 5.0_tile + 1.0_in}}},
    {{{2.5_tile, 1.5_tile}, {3.5_tile, 0.0_tile}}, {{2.5_tile, 4.5_tile}, {3.5_tile, 6.0_tile}}}};
  GoalLandmarker goal_landmarker{{{{3.0_tile, 3.0_tile}, 5.65_in},
                                  {{5.65_in, 5.65_in}, 5.65_in},
                                  {{3.0_tile, 5.65_in}, 5.65_in},
                                  {{6.0_tile - 5.65_in, 5.65_in}, 5.65_in},
                                  {{5.65_in, 3.0_tile}, 5.65_in},
                                  {{6.0_tile - 5.65_in, 3.0_tile}, 5.65_in},
                                  {{5.65_in, 6.0_tile - 5.65_in}, 5.65_in},
                                  {{3.0_tile, 6.0_tile - 5.65_in}, 5.65_in},
                                  {{6.0_tile - 5.65_in, 6.0_tile - 5.65_in}, 5.65_in}}};
};
} // namespace bfb