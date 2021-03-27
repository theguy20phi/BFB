#pragma once

#include "okapi/api/control/util/settledUtil.hpp"
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QLength.hpp"
#include "okapi/api/units/QSpeed.hpp"
#include "okapi/impl/util/timeUtilFactory.hpp"
#include <memory>

using namespace okapi::literals;
namespace bfb {
using SettledUtilPtr = std::unique_ptr<okapi::SettledUtil>;
SettledUtilPtr make_settled_util(double i_max_error = 50,
                                 double i_max_deriv = 5,
                                 okapi::QTime i_min_time = 250_ms);

enum class Color { Blue, Red };

constexpr okapi::QSpeed inps = okapi::inch / okapi::second;

inline namespace literals {
constexpr okapi::QSpeed _inps(long double x) {
  return x * inps;
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