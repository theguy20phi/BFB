#pragma once

#include "crossOdometry.hpp"
#include "poseEstimator.hpp"

namespace bfb {
class ThreeEncoderOdometry : public PoseEstimator {
  public:
  /**
   * @brief Construct a new Cross Odometry object
   *
   * @param iLeftOdometer
   * @param iSideOdometer
   * @param iImus
   */
  ThreeEncoderOdometry(const Odometer &iLeftOdometer,
                const Odometer &iRightOdometer,
                const Odometer &iSideOdometer,
                const IMU &iImus);

  void updatePose() override;

  /**
   * @brief Get the estimated pose according to odometry.
   * @return Pose
   */
  Pose getPose() override;

  /**
   * @brief Resets the odometry.
   *
   * @param iPose
   */
  void setPose(const Pose &iPose) override;

  /**
   * @brief Performs a basic reset of odometry (clearing pose + resetting sensors)
   */
  void reset() override;

  private:
  Pose pose{0.0_in, 0.0_in, 0.0_rad};
  Odometer leftOdometer;
  Odometer rightOdometer;
  Odometer sideOdometer;
  const double gyroThresh{0.075};
  IMU imus;
  double previousH{0};
};
} // namespace bfb