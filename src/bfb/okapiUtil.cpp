#include "okapiUtil.hpp"

namespace bfb {
SettledUtilPtr make_settled_util(double i_max_error, double i_max_deriv, okapi::QTime i_min_time) {
  return std::make_unique<okapi::SettledUtil>(
    okapi::TimeUtilFactory::createDefault().getTimer(), i_max_error, i_max_deriv, i_min_time);
}
} // namespace bfb