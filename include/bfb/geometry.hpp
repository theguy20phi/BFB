#pragma once

#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QAngularSpeed.hpp"
#include "okapi/api/units/QLength.hpp"
#include "okapi/api/units/QSpeed.hpp"
#include "mathUtil.hpp"
namespace bfb {
struct Point {
  okapi::QLength x{0.0};
  okapi::QLength y{0.0};
  okapi::QAngle h{0.0};
  okapi::QLength distance_to(const Point &other) const;
  Point rotate(const okapi::QAngle &ccw) const;
};

struct LineSegment {
    Point a;
    Point b;
    okapi::QLength distance_to(const Point &point) const;
    okapi::QLength distance_in_bounds(const Point &point) const;
    bool is_in_bounds(const Point &point) const;
};

struct Pose : public Point {
  okapi::QSpeed v{0.0};
  okapi::QAngularSpeed w{0.0};
};

struct Circle {
  Point center;
  okapi::QLength radius;
  okapi::QLength distance_to(const Point &point) const;
};

struct Rectangle {
  Point left_top;
  Point right_bot;
  bool is_in_perimeter(const Point &point) const;
};
} // namespace bfb