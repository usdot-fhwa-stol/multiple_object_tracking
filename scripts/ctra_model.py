import math
import matplotlib.pyplot as plt

class CtraState(object):
    def __init__(self, state):
        self.pos_x = state[0]
        self.pos_y = state[1]
        self.velocity = state[2]
        self.yaw = state[3]
        self.yaw_rate = state[4]
        self.acceleration = state[5]

    def __str__(self):
        text = f"Current State: \n  x: {self.pos_x}\n  y: {self.pos_y}\n  velocity: {self.velocity}\n  yaw: {self.yaw}\n  yaw_rate: {self.yaw_rate}\n  acceleration: {self.acceleration}\n"
        return text

def nextState(state : CtraState, time_step):
    yaw_rate = state.yaw_rate
    acceleration = state.acceleration
    velocity_new = state.velocity + acceleration * time_step
    yaw_new = state.yaw + yaw_rate * time_step

    if (state.yaw_rate == 0):
        delta_pos_x = (state.velocity * time_step + (acceleration*time_step*time_step)/2) * math.cos(state.yaw)
        delta_pos_y = (state.velocity * time_step + (acceleration*time_step*time_step)/2) * math.sin(state.yaw)
        # delta_pos_x = state.velocity * math.cos(state.yaw) * time_step;
        # delta_pos_y = state.velocity * math.sin(state.yaw) * time_step;

    else:
        delta_pos_x = (1/(yaw_rate * yaw_rate)) * (velocity_new * yaw_rate * math.sin(yaw_new) + acceleration * math.cos(yaw_new) - state.velocity * yaw_rate * math.sin(state.yaw) - acceleration * math.cos(state.yaw))

        delta_pos_y = (1/(yaw_rate * yaw_rate)) * (-velocity_new * yaw_rate * math.cos(yaw_new) + acceleration * math.sin(yaw_new) + state.velocity * yaw_rate * math.cos(state.yaw) - acceleration * math.sin(state.yaw))

    pos_x_new = state.pos_x + delta_pos_x
    pos_y_new = state.pos_y + delta_pos_y
    new_state = [pos_x_new, pos_y_new, velocity_new, yaw_new, yaw_rate, acceleration]
    return CtraState(new_state)


# state = [0, 0, 10, -4.3, 1, 1]
# n = CtraState(state)
# time_step = 0.5
# x = []
# y = []

# for i in range(0, 100):
#     x.append(n.pos_x)
#     y.append(n.pos_y)
#     n = nextState(n, time_step)
#     # print(f"{i}.) {n}")

# plt.plot(x, y)
# plt.show()

# Test for CTRA model:
#  Test CTRA nextState function against pure rotation
state = [0, 0, 0, 0, 1, 0]
time_step = 0.5
n = CtraState(state)
n = nextState(n, time_step)
print(f"Test 1.) {n}")

# Test for CTRA model:
# Test CTRA nextState function against pure translation
state = [0, 0, 1, 0, 0, 1]
time_step = 0.5
n = CtraState(state)
n = nextState(n, time_step)
print(f"Test 2.) {n}")

# Test for CTRA model:
# Test CTRA nextState function against mixed rotation and translation
state = [0, 0, 1, 0, 1, 1]
time_step = 0.5
n = CtraState(state)
n = nextState(n, time_step)
print(f"Test 3.) {n}")
