import numpy as np


class Point:
    def __init__(self, i_x, i_y, i_h=0.0):
        self.x = i_x
        self.y = i_y
        self.h = i_h

    def distance_to(self, other):
        return np.sqrt((self.x - other.x)**2.0 + (self.y - other.y)**2.0)


class LineSegment:
    def __init__(self, i_a, i_b):
        self.a = i_a
        self.b = i_b

    def distance_to(self, point):
        if self.is_in_bounds(point):
            return self._distance_in_bounds(point)
        return min([point.distance_to(self.a), point.distance_to(self.b)])

    def _distance_in_bounds(self, point):
        a_b_x_diff = self.b.x - self.a.x
        a_b_y_diff = self.b.y - self.a.y
        left_numerator = a_b_x_diff * (self.a.y - point.y)
        right_numerator = (self.a.x - point.x) * a_b_y_diff
        numerator = abs(left_numerator - right_numerator)
        denominator = np.sqrt(a_b_x_diff**2.0 + a_b_y_diff**2.0)
        return numerator / denominator

    def is_in_bounds(self, point):
        if self.a.x == self.b.x:
            return is_between(point.y, self.a.y, self.b.y)
        if self.a.y == self.b.y:
            return is_between(point.x, self.a.x, self.b.x)
        slope = (self.b.y - self.a.y) / (self.b.x - self.a.x)
        inverse_slope = -1.0 / slope
        lower = inverse_slope * (point.x - self.a.x) + self.a.y
        upper = inverse_slope * (point.x - self.b.x) + self.b.y
        return is_between(point.y, lower, upper)


def is_between(value, left, right):
    return (value < right and value > left) or (value < left and value > right)


class Circle:
    def __init__(self, i_center, i_radius):
        self.center = i_center
        self.radius = i_radius

    def distance_to(self, point):
        return self.center.distance_to(point)


class Rectangle:
    def __init__(self, i_left_top, i_right_bot):
        self.left_top = i_left_top
        self.right_bot = i_right_bot

    def is_in_perimeter(self, point):
        return is_between(point.x, self.left_top.x, self.right_bot.x) and is_between(point.y, self.left_top.y, self.right_bot.y)


class LineSegmentLandmarker:
    def __init__(self, i_line_segments, i_exclusion_zone=Rectangle(Point(0, 0), Point(0, 0)), i_probability=1.0):
        self.line_segments = i_line_segments
        self.exclusion_zone = i_exclusion_zone
        self.probability = i_probability
        self.comp_probability = 1.0 - self.probability

    def correct_position(self, current_position, signal):
        if signal and not self.exclusion_zone.is_in_perimeter(current_position):
            self.line_segments.sort(
                key=lambda seg: seg.distance_to(current_position))
            closest_line_segment = self.line_segments[0]
            return self._weight(current_position, self._get_closest_point(current_position, closest_line_segment))
        return current_position

    def _get_closest_point(self, current_position, closest_line_segment):
        if closest_line_segment.is_in_bounds(current_position):
            if closest_line_segment.a.x == closest_line_segment.b.x:
                return Point(closest_line_segment.a.x, current_position.y, current_position.h)
            if closest_line_segment.a.y == closest_line_segment.b.y:
                return Point(current_position.x, closest_line_segment.a.y, current_position.h)
            slope = (closest_line_segment.b.y - closest_line_segment.a.y) / \
                (closest_line_segment.b.x - closest_line_segment.a.x)
            x = (slope**2.0 * closest_line_segment.a.x - slope * closest_line_segment.a.y +
                 current_position.x + slope * current_position.y) / (slope**2.0 + 1.0)
            y = slope * x - slope * closest_line_segment.a.x + closest_line_segment.a.y
            return Point(x, y)
        return closest_line_segment.a if closest_line_segment.a.distance_to(current_position) < closest_line_segment.b.distance_to(current_position) else closest_line_segment.b

    def _weight(self, current_position, proposed_position):
        return Point(current_position.x * self.comp_probability + proposed_position.x * self.probability, current_position.y * self.comp_probability + proposed_position.y * self.probability, current_position.h)


class GoalLandmarker:
    def __init__(self, i_goals, i_probability=1.0):
        self.goals = i_goals
        self.probability = i_probability
        self.comp_probability = 1.0 - self.probability

    def correct_position(self, current_position, signal):
        if signal:
            self.goals.sort(key=lambda goal: goal.distance_to(current_position))
            closest_goal = self.goals[0]
            return self._weight(current_position, self._get_closest_point(current_position, closest_goal))
        return current_position
    
    def _get_closest_point(self, current_position, closest_goal):
        x_diff = current_position.x - closest_goal.center.x
        y_diff = current_position.y - closest_goal.center.y
        x = closest_goal.center.x + closest_goal.radius * (x_diff / np.sqrt(x_diff**2.0 + y_diff**2.0))
        y = closest_goal.center.y + closest_goal.radius * (y_diff / np.sqrt(x_diff**2.0 + y_diff**2.0))
        h = np.arctan2(closest_goal.center.y - y, closest_goal.center.x - x)
        return Point(x, y, h)

    def _weight(self, current_position, proposed_position):
        return Point(current_position.x * self.comp_probability + proposed_position.x * self.probability, current_position.y * self.comp_probability + proposed_position.y * self.probability, current_position.h * self.comp_probability + proposed_position.h * self.probability)


a = Point(0.0, 0.0)
b = Point(10.0, 10.0)
c = Point(0.0, 10.0)
z = Point(-10.0, -5.0, 45.0)
exclude = Rectangle(Point(-9.0, -3.0), Point(-5.0, -7.0))
l1 = LineSegment(a, b)
l2 = LineSegment(a, c)
ls_landmarker = LineSegmentLandmarker([l1, l2], exclude, 0.9)
z = ls_landmarker.correct_position(z, False)
print(exclude.is_in_perimeter(z))
print(f"{z.x}, {z.y}, {z.h}")

p = Point(2.0, 2.0)
goal1 = Circle(Point(0.0, 0.0), 2.0)
goal2 = Circle(Point(5.0, 5.0), 1.0)
g_landmarker = GoalLandmarker([goal1, goal2])
p = g_landmarker.correct_position(p, True)
print(f"{p.x}, {p.y}, {p.h * 180.0 / 3.14159265}")