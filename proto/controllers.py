import numpy as np
import matplotlib.pyplot as plt

dt = 0.01
command_to_volt = 0.0

class PID:
    def __init__(self, i_k_p, i_k_i, i_k_d):
        self.k_p = i_k_p
        self.k_i = i_k_i
        self.k_d = i_k_d
        self.target = 0.0
        self.output = 0.0
        self.i = 0.0
        self.previous_state = 0.0
        self.i_decay = 0.95

    def calculate(self, state):
        error = self.target - state
        p = self.k_p * error
        self.i += self.k_i * error
        if np.sign(error) != np.sign(self.target - self.previous_state):
            self.i = 0.0
        self.i *= self.i_decay
        d = self.k_d * (self.previous_state - state)
        self.output = p + self.i + d
        self.previous_state = state

    def get_output(self):
        return self.output

    def set_target(self, i_target):
        self.target = i_target

class TBH: 
    def __init__(self, i_k_i):
        self.k_i = i_k_i
        self.target = 0.0
        self.output = 0.0
        self.tbh = 0.0
        self.previous_error = 0.0

    def calculate(self, state):
        error = self.target - state
        self.output += self.k_i * error
        if(np.sign(error) != np.sign(self.previous_error)):
            self.output += self.tbh
            self.output *= 0.5
            self.previous_error = error
    
    def get_output(self):
        return self.output

    def set_target(self, i_target):
        self.target = i_target
        self.tbh = command_to_volt * self.target


class PhysicsObject:
    def __init__(self, i_mass, i_friction, i_dt):
        self.mass = i_mass
        self.friction = i_friction
        self.dt = i_dt
        self.position = 0.0
        self.velocity = 0.0

    def act(self, force):
        accel = force / self.mass - self.friction * np.sign(self.velocity)
        self.position += self.velocity * self.dt + 0.5 * accel * self.dt ** 2.0
        self.velocity += accel * self.dt
        return self.position

    def get_position(self):
        return self.position


def clamp(a, b, c):
    if a < b:
        return b
    if a > c:
        return c
    return a

target = 10
arm = PhysicsObject(1, 10, dt)
controller_a = PID(175.0, 0.0001, 0.0)
controller_a.set_target(target)
ps = []
os = []
ts = range(200)
for i in ts:
    controller_a.calculate(arm.velocity)
    force = clamp(controller_a.get_output(), -25 , 25)
    arm.act(force)
    ps.append(arm.velocity)
    os.append(controller_a.get_output())

p_fig, p_ax = plt.subplots()
p_ax.set_xlabel("Step (#)")
p_ax.set_ylabel("Position (#)")
p_ax.plot(ts, ps, ts, [target for i in ts])

o_ax = p_ax.twinx()
o_ax.set_ylabel("Output")
o_ax.plot(ts, os, "tab:red")

plt.show()
