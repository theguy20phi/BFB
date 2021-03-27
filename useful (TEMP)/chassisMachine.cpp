#include "chassisMachine.hpp"

ChassisMachine::ChassisMachine(const Chassis::ChassisStates &iState) : StateMachine(iState) {
  auto odom = bfb::ThreeEncoderOdometryBuilder()
                .withLeftOdometer(pros::ADIEncoder{8, 7, true}, 2.75, 8.63)
                .withRightOdometer(pros::ADIEncoder{4, 3, true}, 1.75, 8.63)
                .withSideOdometer(pros::ADIEncoder{5, 6, true}, 0.4, 8.63)
                .withIMUs({13, 14, 15});
  poseEstimator = bfb::CPEBuilder().withEstimator(odom.build(), 1.0).build();
  coast();
  poseEstimator->reset();
}

void ChassisMachine::behavior(const Chassis::Standby &standby) {
  moveVelocity(0.0, 0.0, 0.0);
}

void ChassisMachine::behavior(const Chassis::Control &control) {
  controlDrive(control.forward, control.strafe, control.turn);
}

void ChassisMachine::behavior(const Chassis::MoveTo &moveTo) {
  planStep(moveTo);
  controlDrive(yPidf.getOutput(), xPidf.getOutput(), hPidf.getOutput());
}

void ChassisMachine::controlDrive(double forward, double strafe, double turn) {
  if (fabs(forward) + fabs(strafe) + fabs(turn) < deadband)
    moveVelocity(0.0, 0.0, 0.0);
  else
    moveVoltage(forward, strafe, turn);
}

void ChassisMachine::moveVelocity(double forward, double strafe, double turn) {
  lFWheel.move_velocity(forward + strafe + turn);
  lBWheel.move_velocity(forward - strafe + turn);
  rFWheel.move_velocity(forward - strafe - turn);
  rBWheel.move_velocity(forward + strafe - turn);
}

void ChassisMachine::moveVoltage(double forward, double strafe, double turn) {
  lFWheel.move_voltage(forward + strafe + turn);
  lBWheel.move_voltage(forward - strafe + turn);
  rFWheel.move_voltage(forward - strafe - turn);
  rBWheel.move_voltage(forward + strafe - turn);
}

void ChassisMachine::planStep(const Chassis::MoveTo &moveTo) {
  const double xDiff{(poseEstimator->getPose().x - moveTo.x).convert(okapi::inch)};
  const double yDiff{(poseEstimator->getPose().y - moveTo.y).convert(okapi::inch)};
  const double distance{sqrt(xDiff * xDiff + yDiff * yDiff)};
  const double direction{poseEstimator->getPose().h.convert(okapi::radian) + atan2(yDiff, xDiff)};
  const double xDistance{distance * cos(direction)};
  const double yDistance{distance * sin(direction)};
  const double hDistance{okapi::OdomMath::constrainAngle180(poseEstimator->getPose().h - moveTo.h)
                           .convert(okapi::radian)};
  xPidf.calculate(xDistance);
  yPidf.calculate(yDistance);
  hPidf.calculate(hDistance);
  if (xPidf.isDone(xDistance) && yPidf.isDone(yDistance) && hPidf.isDone(hDistance))
    setState(Chassis::Standby{});
}

okapi::QLength ChassisMachine::X() const {
  return poseEstimator->getPose().x;
}

okapi::QLength ChassisMachine::Y() const {
  return poseEstimator->getPose().y;
}

okapi::QAngle ChassisMachine::H() const {
  return poseEstimator->getPose().h;
}

void ChassisMachine::setPose(const bfb::Pose &iPose) {
  for (int i = 0; i < 5; i++) {
    poseEstimator->setPose(iPose);
    setState(Chassis::Standby{});
    bfb::wait(10);
  }
}

void ChassisMachine::reset() {
  for (int i = 0; i < 5; i++) {
    poseEstimator->reset();
    bfb::wait(10);
    setState(Chassis::Standby{});
  }
}

void ChassisMachine::toggleHold() {
  if (lFWheel.get_brake_mode() == pros::E_MOTOR_BRAKE_COAST) {
    hold();
  } else {
    coast();
  }
}

void ChassisMachine::hold() {
  lFWheel.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  lBWheel.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  rFWheel.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  rBWheel.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

void ChassisMachine::coast() {
  lFWheel.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  lBWheel.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  rFWheel.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  rBWheel.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}