/**
 * @file chassisMachine.hpp
 * @author Braden Pierce (913153006@bryantschools.org)
 *
 * @copyright Copyright (c) 2020
 *
 */

#pragma once

#include "main.h"

namespace Chassis {
/**
 * @brief Standby state.
 *
 */
struct Standby {};

/**
 * @brief Control state, made of three integers to represent forward, strafe, and turn commands.
 *
 */
struct Control {
  int forward;
  int strafe;
  int turn;
};

/**
 * @brief MoveTo State, made of two QLengths (x and y), and a QAngle (h).
 *
 */
struct MoveTo {
  okapi::QLength x;
  okapi::QLength y;
  okapi::QAngle h;
};

using ChassisStates = std::variant<Standby, Control, MoveTo>;
} // namespace Chassis

/**
 * @brief Represents the entire chassis of the robot and is responsible for its behavior during
 * auton and driving.
 *
 */
class ChassisMachine : public bfb::StateMachine<ChassisMachine, Chassis::ChassisStates> {
  public:

  /**
   * @brief Resets the internal pose of the odometry.
   *
   */
  void reset();

  private:
  void controlDrive(double forward, double strafe, double turn);
  void moveVelocity(double forward, double strafe, double turn);
  void moveVoltage(double forward, double strafe, double turn);
  void planStep(const Chassis::MoveTo &moveTo);
  void brake();

  private:
  bfb::PoseEstimatorPtr poseEstimator;
  pros::Motor lFWheel{11};
  pros::Motor lBWheel{20};
  pros::Motor rFWheel{12, true};
  pros::Motor rBWheel{18, true};
  const double deadband{100.0};
  bfb::Pidf xPidf{{2000.0, 25.0}, bfb::createSettledUtil(1.3, 0.3)};
  bfb::Pidf yPidf{{2000.0, 25.0}, bfb::createSettledUtil(1.3, 0.3)};
  bfb::Pidf hPidf{{45000.0, 400.0}, bfb::createSettledUtil(0.06, 0.01)};
};