#include "mathUtil.hpp"

namespace bfb {
bool is_almost_equal(double a, double b, double thresh) {
  if (std::abs(a - b) <= thresh) {
    return true;
  }
  return false;
}

bool is_almost_zero(double a, double thresh) {
  return is_almost_equal(a, 0.0, thresh);
}

double normalize_angle(double angle) {
  return angle - 2.0 * M_PI * std::floor((angle + M_PI) / (2 * M_PI));
}
} // namespace bfb