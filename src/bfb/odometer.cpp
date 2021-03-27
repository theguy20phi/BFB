#include "odometer.hpp"

namespace bfb {
Odometer::Odometer(char port,
                   const okapi::QLength i_distance_to,
                   bool reversed,
                   const okapi::QLength &i_circumference)
  : encoder(port, reversed), distance_to(i_distance_to), circumference(i_circumference) {
}

okapi::QLength Odometer::distance_since() {
  const int reading{encoder.get_value()};
  const okapi::QLength distance{(reading - previous_reading) / 360.0 * circumference};
  previous_reading = reading;
  return distance;
}

okapi::QLength Odometer::distance_to_wheel() const {
  return distance_to;
}

int Odometer::get_raw() const {
  return encoder.get_value();
}

void Odometer::reset() {
  encoder.reset();
  previous_reading = 0;
}
} // namespace bfb