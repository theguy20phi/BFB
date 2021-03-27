#pragma once

#include "mathUtil.hpp"
#include "okapi/api.hpp"
#include "wait.hpp"
#include <assert.h>

namespace bfb {
template <typename P, typename V, typename A> class MotionProfiler {
  public:
  struct Target {
    P distance;
    V velocity;
  };

  MotionProfiler(const P i_threshold, okapi::QTime i_dt = general_delay * okapi::millisecond);

  Target next_target(const Target &current,
                     const Target &target,
                     const V &max_velocity,
                     const A &max_accel) {
    // NOTE Assert may not work at all. Assert may not show message. If need be, this can be removed at the price of idiot proofing. 
    preconditions(target, max_velocity, max_accel);
    // NOTE The sign procedure used here involves comparing a measurement to a generic number 0, it may or may not work. 
    A accel = max_accel * sign(target.distance - current.distance);
    if (is_at_target(current, target))
      return target;
    if (need_to_decelerate(current, target, max_accel))
      return Target{current.distance + current.velocity * dt - 0.5 * accel * dt * dt,
                    current.velocity - accel * dt};
    if (okapi::abs(current.velocity) < max_velocity)
      return Target{current.distance + current.v * dt + 0.5 * accel * dt * dt,
                    current.velocity + accel * dt};
    return Target{current.distance + current.velocity * dt, current.velocity};
  }

  bool is_at_target(const Target &current, const Target &target) {
    if (okapi::abs(target.distance - current.distance) <= threshold)
      return true;
    return false;
  }

  void set_threshold(const P &i_threshold) {
    threshold = i_threshold;
  }

  P get_threshold() const {
    return threshold;
  }

  private:
  void preconditions(const Target &target, const V &max_velocity, const A &max_accel) {
    assert(("Target velocity must be less than or equal to maximum velocity.",
            target.velocity <= max_velocity));
    assert(("Max velocity must be greater than zero.", max_velocity > 0.0));
    assert(("Max acceleration must be greater than zero.", max_accel > 0.0));
  }

  bool need_to_decelerate(const Target &current, const Target &target, const A &max_accel) {
    return (current.velocity * current.velocity - target.velocity * target.velocity) /
             (2.0 * max_accel) >=
           okapi::abs(target.distance - current.distance);
  }

  private:
  P threshold;
  const okapi::QTime dt;
};
} // namespace bfb