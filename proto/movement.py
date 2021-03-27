import matplotlib.pyplot as plt
import numpy as np

dt = 0.01


class Motion:
    def __init__(self, iP, iV):
        self.p = iP
        self.v = iV


class MotionProfiler:
    def __init__(self, i_threshold, i_dt):
        self.threshold = i_threshold
        self.dt = i_dt

    def next_target(self, current, target, max_v=0.1, max_a=0.005):
        self._preconditions(target, max_v, max_a)
        accel = max_a * np.sign(target.p - current.p)

        if abs(target.p - current.p) <= self.threshold:
            return None

        if (current.v**2.0 - target.v**2.0) / (2.0 * max_a) >= abs(target.p - current.p):
            return Motion(current.p + current.v * self.dt - 0.5 * accel * self.dt**2.0, current.v - accel * self.dt)

        if abs(current.v) < max_v:
            return Motion(current.p + current.v * self.dt + 0.5 * accel * self.dt**2.0, current.v + accel * self.dt)
            
        return Motion(current.p + current.v * self.dt, current.v)

    def _preconditions(self, target, max_v, max_a):
        assert target.v <= max_v, "Target velocity must be less than or equal to maximum velocity"
        assert max_v > 0.0, "Max velocity must be greater than zero"
        assert max_a > 0.0, "Max acceleration must be greater than zero"

    def set_threshold(self, i_threshold):
        self.threshold = i_threshold

    def set_dt(self, i_dt):
        self.dt = i_dt

    def get_threshold(self):
        return self.threshold

    def get_dt(self):
        return self.dt


profiler = MotionProfiler(0.001, dt)
c = Motion(0.0, 0.0)
t = Motion(10.0, 0.05)
ps = []
vs = []
demo_a = True
demo_b = True
demo_c = False
demo_d = False
if demo_a:
    while c is not None:
        c = profiler.next_target(c, t)
        if c is None:
            print("Nothin! This is when we would know to move onto the next waypoint.")
            break
        ps.append(c.p)
        vs.append(c.v)
    distance = sum([i * dt for i in vs])
    print(distance)

if demo_b:
    c = t
    t = Motion(0.0, 0.0)
    while c is not None:
        c = profiler.next_target(c, t)
        if c is None:
            print("Nothin! This is when we would know to move onto the next waypoint.")
            break
        ps.append(c.p)
        vs.append(c.v)
    distance = sum([i * dt for i in vs])
    print(distance)

if demo_c:
    c = t
    t = Motion(10.0, 0.025)
    while c is not None:
        c = profiler.next_target(c, t, 0.05, 0.0025)
        if c is None:
            print("Nothin! This is when we would know to move onto the next waypoint.")
            break
        ps.append(c.p)
        vs.append(c.v)
    distance = sum([i * dt for i in vs])
    print(distance)

if demo_d:
    c = t
    t = Motion(0.0, -0.025)
    while c is not None:
        c = profiler.next_target(c, t, 0.05, 0.0025)
        if c is None:
            print("Nothin! This is when we would know to move onto the next waypoint.")
            break
        ps.append(c.p)
        vs.append(c.v)
    distance = sum([i * dt for i in vs])
    print(distance)

ts = [i * dt for i in range(len(vs))]
p_fig, p_ax = plt.subplots()
p_ax.set_xlabel("Time (s)")
p_ax.set_ylabel("Position (u)")
p_ax.plot(ts, ps)

v_ax = p_ax.twinx()
v_ax.set_ylabel("Velocity (u/s)")
v_ax.plot(ts, vs, "tab:red")

plt.show()
