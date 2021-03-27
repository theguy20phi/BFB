#pragma once

#include "bfb/devices/threeEncoderOdometry.hpp"

namespace bfb {
class ThreeEncoderOdometryBuilder {
  public:
  ThreeEncoderOdometryBuilder();

  PoseEstimatorPtr build() const;

  ThreeEncoderOdometryBuilder withLeftOdometer(const pros::ADIEncoder &encoder, double displacement, double circum = 8.639379);
  ThreeEncoderOdometryBuilder withRightOdometer(const pros::ADIEncoder &encoder, double displacement, double circum = 8.639379);
  ThreeEncoderOdometryBuilder withSideOdometer(const pros::ADIEncoder &encoder, double displacement, double circum = 8.639379);
  ThreeEncoderOdometryBuilder withIMUs(const std::vector<uint8_t> &ports);

  private:
  std::shared_ptr<Odometer> leftOdometer;
  std::shared_ptr<Odometer> rightOdometer;
  std::shared_ptr<Odometer> sideOdometer;
  std::shared_ptr<IMU> imus;
};
} // namespace bfb