#include "lineLandmarker.hpp"

namespace bfb {
LineLandmarker::LineLandmarker(const Lines &iLines,
                               const Pose &iRelativePose,
                               uint8_t port,
                               int iThreshold)
  : lines(iLines), relativePose(iRelativePose), lineSensor(port), threshold(iThreshold) {
}

void LineLandmarker::updatePose(const Pose &iReferencePose) {
  pose = iReferencePose;
  const Line closestLine{getClosestLine()};
  calculatePose(closestLine);
}

void LineLandmarker::setPose(const Pose &iReferencePose) {
  pose = iReferencePose;
}

bool LineLandmarker::isReading() {
  if (lineSensor.get_value() < threshold)
    return true;
  return false;
}

Pose LineLandmarker::getPose() {
  return pose;
}

void LineLandmarker::reset() {
  pose = {0.0_in, 0.0_in, 0.0_rad};
}

Line LineLandmarker::getClosestLine() const {
  Line closestLine{lines[0]};
  double shortestDistance{calculateDistance(closestLine)};
  for (Line line : lines) {
    const double distance{calculateDistance(line)};
    if (distance >= shortestDistance)
      continue;
    closestLine = line;
    shortestDistance = distance;
  }
  return closestLine;
}

double LineLandmarker::calculateDistance(const Line &line) const {
  if (line.orientation == Orientation::horizontal)
    return okapi::abs(line.distance - pose.y).convert(okapi::meter);
  else
    return okapi::abs(line.distance - pose.x).convert(okapi::meter);
}

void LineLandmarker::calculatePose(const Line &line) {
  if (!isReading())
    return;
  if (line.orientation == Orientation::horizontal)
    pose = {pose.x, line.distance - relativePose.y * cos(pose.h), pose.h};
  else
    pose = {line.distance - relativePose.x * sin(pose.h), pose.y, pose.h};
}
} // namespace bfb