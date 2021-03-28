#pragma once

#include "okapi/api.hpp"
#include <memory>

using namespace okapi::literals;

namespace bfb {
using SettledUtilPtr = std::unique_ptr<okapi::SettledUtil>;
SettledUtilPtr make_settled_util(double i_max_error = 50,
                                 double i_max_deriv = 5,
                                 okapi::QTime i_min_time = 250_ms);

enum class Color { Blue, Red };

constexpr okapi::QSpeed inps = okapi::inch / okapi::second;
constexpr okapi::QAcceleration inpsps = okapi::inch / (okapi::second * okapi::second);
constexpr okapi::QAngularAcceleration radpsps = okapi::radian / (okapi::second * okapi::second);

inline namespace literals {
constexpr okapi::QSpeed operator""_inps(long double x) {
  return x * inps;
}
constexpr okapi::QAcceleration operator""_inpsps(long double x) {
  return x * inpsps;
}
constexpr okapi::QAngularSpeed operator""_radps(long double x) {
  return x * okapi::radps;
}
constexpr okapi::QAngularAcceleration operator""_radpsps(long double x) {
  return x * radpsps;
}
constexpr okapi::QLength operator""_tile(long double x) {
  return x * 24.0 * okapi::inch;
}
constexpr okapi::QLength operator""_element(long double x) {
  return x * 6.3 * okapi::inch;
}
constexpr okapi::QLength operator""_bot_width(long double x) {
  return x * 17.5 * okapi::inch;
}
constexpr okapi::QLength operator""_bot_length(long double x) {
  return x * 17.5 * okapi::inch;
}
} // namespace literals
} // namespace bfb