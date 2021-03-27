#include "threeEncoderOdometry.hpp"

namespace bfb {
ThreeEncoderOdometry::ThreeEncoderOdometry(const Odometer &iLeftOdometer,
                                           const Odometer &iRightOdometer,
                                           const Odometer &iSideOdometer,
                                           const IMU &iImus)
  : leftOdometer(iLeftOdometer),
    rightOdometer(iRightOdometer),
    sideOdometer(iSideOdometer),
    imus(iImus) {
  imus.calibrate();
  bfb::wait(500);
  bfb::waitUntil([=]() { return !imus.isCalibrating(); }, 6000);
}

void ThreeEncoderOdometry::updatePose() {
  const double tempH{toRadians(imus.getHeading())};
  const double deltaGH{tempH - previousH};
  const double lODelta{leftOdometer.getDelta()};
  const double rODelta{rightOdometer.getDelta()};
  const double deltaOH{(lODelta + rODelta) / (leftOdometer.getDisp() + rightOdometer.getDisp())};
  const double deltaH{fabs(deltaGH - deltaOH) > gyroThresh ? deltaGH : deltaOH};
  double localX{0.0};
  double localY{0.0};
  if (deltaH) {
    double twoSinHalfDeltaH{2.0 * sin(deltaH / 2.0)};
    localX = twoSinHalfDeltaH * (sideOdometer.getDelta() / deltaH + sideOdometer.getDisp());
    localY =
      twoSinHalfDeltaH * 0.5 *
      (rODelta / deltaH + rightOdometer.getDisp() + lODelta / deltaH + leftOdometer.getDisp());
  } else {
    localX = sideOdometer.getDelta();
    localY = (rODelta + lODelta) / 2.0;
  }
  std::cout << lODelta << " and " << rODelta << std::endl;
  std::cout << pose.x.convert(okapi::inch) << ", " << pose.y.convert(okapi::inch) << ", " << pose.h.convert(okapi::degree) << std::endl;
  const double magnitude{sqrt(localX * localX + localY * localY)};
  const double direction{atan2(localY, localX) - (previousH + deltaH / 2.0)};
  double deltaX{cos(direction) * magnitude};
  double deltaY{sin(direction) * magnitude};
  pose.x += deltaX * okapi::inch;
  pose.y += deltaY * okapi::inch;
  pose.h = tempH * okapi::radian;
  previousH = tempH;
}

Pose ThreeEncoderOdometry::getPose() {
  return pose;
}

void ThreeEncoderOdometry::setPose(const Pose &iPose) {
  pose = iPose;
  imus.resetHeading(iPose.h.convert(okapi::degree));
}

void ThreeEncoderOdometry::reset() {
  pose = {0.0_in, 0.0_in, 0.0_rad};
  imus.resetHeading(0.0);
}
} // namespace bfb