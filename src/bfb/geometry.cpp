#include "geometry.hpp"

namespace bfb {
okapi::QLength Point::distance_to(const Point &other) const {
  const auto x_diff_sqrd = (x - other.x) * (x - other.x);
  const auto y_diff_sqrd = (y - other.y) * (y - other.y);
  return okapi::sqrt(x_diff_sqrd + y_diff_sqrd);
}

Point Point::rotate(const okapi::QAngle &ccw) const {
  return Point{x * okapi::cos(ccw) - y * okapi::sin(ccw),
               x * okapi::sin(ccw) + y * okapi::cos(ccw)};
}

okapi::QLength LineSegment::distance_to(const Point &point) const {
  if (is_in_bounds(point))
    return distance_in_bounds(point);
  const okapi::QLength distance_to_a{point.distance_to(a)};
  const okapi::QLength distance_to_b{point.distance_to(b)};
  if (distance_to_a < distance_to_b)
    return distance_to_a;
  return distance_to_b;
}

okapi::QLength LineSegment::distance_in_bounds(const Point &point) const {
  const okapi::QLength a_b_x_diff{b.x - a.x};
  const okapi::QLength a_b_y_diff{b.y - a.y};
  const auto left_numerator = a_b_x_diff * (a.y - point.y);
  const auto right_numerator = (a.x - point.x) * a_b_y_diff;
  const auto numerator = okapi::abs(left_numerator - right_numerator);
  const auto denominator = okapi::sqrt(a_b_x_diff * a_b_x_diff + a_b_y_diff * a_b_y_diff);
  return numerator / denominator;
}

bool LineSegment::is_in_bounds(const Point &point) const {
  if (a.y == b.y)
    return is_between(point.x, a.x, b.x);
  const auto inverse_slope = (a.x - b.x) / (b.y - a.y);
  const auto lower = inverse_slope * (point.x - a.x) + a.y;
  const auto upper = inverse_slope * (point.x - b.x) + b.y;
  return is_between(point.y, lower, upper);
}

okapi::QLength Circle::distance_to(const Point &point) const {
  return center.distance_to(point);
}

bool Rectangle::is_in_perimeter(const Point &point) const {
  return is_between(point.x, left_top.x, right_bot.x) &&
         is_between(point.y, left_top.y, right_bot.y);
}
} // namespace bfb