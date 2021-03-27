#include "threeEncoderOdometryBuilder.hpp"

namespace bfb {
ThreeEncoderOdometryBuilder::ThreeEncoderOdometryBuilder() {
}

PoseEstimatorPtr ThreeEncoderOdometryBuilder::build() const {
  return PoseEstimatorPtr{new ThreeEncoderOdometry(*leftOdometer, *rightOdometer, *sideOdometer, *imus)};
}

ThreeEncoderOdometryBuilder
ThreeEncoderOdometryBuilder::withLeftOdometer(const pros::ADIEncoder &encoder,
                                              double displacement,
                                              double circum) {
  leftOdometer = std::make_shared<Odometer>(encoder, displacement, circum);
  return *this;
}

ThreeEncoderOdometryBuilder
ThreeEncoderOdometryBuilder::withRightOdometer(const pros::ADIEncoder &encoder,
                                              double displacement,
                                              double circum) {
  rightOdometer = std::make_shared<Odometer>(encoder, displacement, circum);
  return *this;
}

ThreeEncoderOdometryBuilder
ThreeEncoderOdometryBuilder::withSideOdometer(const pros::ADIEncoder &encoder,
                                              double displacement,
                                              double circum) {
  sideOdometer = std::make_shared<Odometer>(encoder, displacement, circum);
  return *this;
}

ThreeEncoderOdometryBuilder ThreeEncoderOdometryBuilder::withIMUs(const std::vector<uint8_t> &ports) {
  imus = std::make_shared<IMU>(ports);
  return *this;
}
} // namespace bfb