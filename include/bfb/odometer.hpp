#pragma once

#include "okapi/api/units/QLength.hpp"
#include "pros/adi.hpp"

using namespace okapi::literals;

namespace bfb {
class Odometer {
  public:
  Odometer(char port,
           const okapi::QLength i_distance_to,
           bool reversed = false,
           const okapi::QLength &i_circumference = 8.639379797371932_in);
  okapi::QLength distance_since();
  okapi::QLength distance_to_wheel() const;
  int get_raw() const;
  void reset();

  private:
  pros::ADIEncoder encoder;
  const okapi::QLength distance_to;
  const okapi::QLength circumference;
  int previous_reading{0};
};
} // namespace bfb