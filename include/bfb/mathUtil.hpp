#pragma once

#include <algorithm>
#include <cmath>

namespace bfb {
template <typename T> struct Range {
  T min;
  T max;
};

template <typename T> T clamp(const T &value, const Range<T> range) {
  if (value < range.min)
    return range.min;
  if (value > range.max)
    return range.max;
  return value;
}

bool is_almost_equal(double a, double b, double thresh);
bool is_almost_zero(double a, double thresh);

template <typename T> int sign(const T &value) {
  if (value < 0)
    return -1;
  else if (value > 0)
    return 1;
  return 0;
}

constexpr double to_radians(double angle) {
  return angle * M_PI / 180.0;
}

constexpr double to_degrees(double angle) {
  return angle * 180.0 / M_PI;
}

//(-PI, PI)
double normalize_angle(double angle);
} // namespace bfb