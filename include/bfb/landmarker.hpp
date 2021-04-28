#pragma once

#include "geometry.hpp"
#include <vector>
#include <iostream>

using namespace okapi::literals;

namespace bfb {
class LineLandmarker {
  public:
  LineLandmarker(const std::vector<LineSegment> &i_line_segments,
                 const std::vector<Rectangle> &i_exclusion_zones,
                 double i_probability = 1.0);
  Pose correct_position(const Pose &current, bool signal) const;

  private:
  bool is_in_exclusion_zone(const Pose &current) const;
  LineSegment get_closest_line_segment(const Pose &current) const;
  Point get_closest_point(const Pose &current, const LineSegment &closest_segment) const;
  Pose weight(const Pose &current, const Point &proposed) const;

  private:
  const std::vector<LineSegment> line_segments;
  const std::vector<Rectangle> exclusion_zones;
  const double probability;
  const double comp_probability;
};

class GoalLandmarker {
  public:
  GoalLandmarker(const std::vector<Circle> &i_goals, double i_probability = 1.0);
  Pose correct_position(const Pose &current, bool signal) const;

  private:
  Circle get_closest_goal(const Pose &current) const;
  Point get_closest_point(const Pose &current, const Circle &closest_goal) const;
  Pose weight(const Pose &current, const Point &proposed) const;

  private:
  const std::vector<Circle> goals;
  const okapi::QLength distance_to;
  const double probability;
  const double comp_probability;
};
} // namespace bfb