#include "landmarker.hpp"

// TODO Using a point procedure from a pose object may cause issues

namespace bfb {
LineLandmarker::LineLandmarker(const std::vector<LineSegment> &i_line_segments,
                               const std::vector<Rectangle> &i_exclusion_zones,
                               double i_probability)
  : line_segments(i_line_segments),
    exclusion_zones(i_exclusion_zones),
    probability(i_probability),
    comp_probability(1.0 - i_probability) {
}

Pose LineLandmarker::correct_position(const Pose &current, bool signal) const {
  if (!signal || is_in_exclusion_zone(current))
    return current;
  const LineSegment closest_segment{get_closest_line_segment(current)};
  return weight(current, get_closest_point(current, closest_segment));
}

bool LineLandmarker::is_in_exclusion_zone(const Pose &current) const {
  bool inside{false};
  for (Rectangle zone : exclusion_zones) {
    if (zone.is_in_perimeter(current)) {
      inside = true;
      break;
    }
  }
  return inside;
}

LineSegment LineLandmarker::get_closest_line_segment(const Pose &current) const {
  LineSegment closest_segment{line_segments[0]};
  okapi::QLength closest_distance{closest_segment.distance_to(current)};
  for (int i = 0; i < line_segments.size(); i++) {
    okapi::QLength distance{line_segments[i].distance_to(current)};
    if (distance < closest_distance) {
      closest_distance = distance;
      closest_segment = line_segments[i];
    }
  }
  return closest_segment;
}

Point LineLandmarker::get_closest_point(const Pose &current,
                                        const LineSegment &closest_segment) const {
  if (closest_segment.is_in_bounds(current)) {
    if (closest_segment.a.x == closest_segment.b.x)
      return Point{closest_segment.a.x, current.y, current.h};
    if (closest_segment.a.y == closest_segment.b.y)
      return Point{current.x, closest_segment.a.y, current.h};
    const auto slope =
      (closest_segment.b.y - closest_segment.a.y) / (closest_segment.b.x - closest_segment.a.x);
    const okapi::QLength x{(slope * slope * closest_segment.a.x - slope * closest_segment.a.y +
                            current.x + slope * current.y) /
                           (slope * slope + 1.0 * okapi::number)};
    const okapi::QLength y{(slope * x - slope * closest_segment.a.x + closest_segment.a.y)};
    return Point{x, y};
  }
  return (closest_segment.a.distance_to(current) < closest_segment.b.distance_to(current))
           ? closest_segment.a
           : closest_segment.b;
}

Pose LineLandmarker::weight(const Pose &current, const Point &proposed) const {
  return Pose{current.x * comp_probability + proposed.x * probability,
              current.y * comp_probability + proposed.y * probability,
              current.h,
              current.v,
              current.w};
}

GoalLandmarker::GoalLandmarker(const std::vector<Circle> &i_goals, const double i_probability)
  : goals(i_goals), probability(i_probability), comp_probability(1.0 - i_probability) {
}

Pose GoalLandmarker::correct_position(const Pose &current, bool signal) const {
  if (!signal)
    return current;
  const Circle closest_goal{get_closest_goal(current)};
  return weight(current, get_closest_point(current, closest_goal));
}

Circle GoalLandmarker::get_closest_goal(const Pose &current) const {
  Circle closest_goal{goals[0]};
  okapi::QLength closest_distance{closest_goal.distance_to(current)};
  for (int i = 0; i < goals.size(); i++) {
    okapi::QLength distance{goals[i].distance_to(current)};
    if (distance < closest_distance) {
      closest_distance = distance;
      closest_goal = goals[i];
    }
  }
  return closest_goal;
}

Point GoalLandmarker::get_closest_point(const Pose &current, const Circle &closest_goal) const {
  const okapi::QLength x_diff{current.x - closest_goal.center.x};
  const okapi::QLength y_diff{current.y - closest_goal.center.y};
  const okapi::QAngle h{90.0_deg - okapi::atan2(y_diff, x_diff) - 180.0_deg};
  const okapi::QLength x{closest_goal.center.x + closest_goal.radius * okapi::cos(270.0_deg - h)};
  const okapi::QLength y{closest_goal.center.y + closest_goal.radius * okapi::sin(270.0_deg - h)};
  return Point{x, y, h};
}

Pose GoalLandmarker::weight(const Pose &current, const Point &proposed) const {
  return Pose{current.x * comp_probability + proposed.x * probability,
              current.y * comp_probability + proposed.y * probability,
              current.h,
              current.v,
              current.w};
}
} // namespace bfb