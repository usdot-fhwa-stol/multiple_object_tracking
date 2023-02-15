import math

class CtrvState(object):
    def __init__(self, state):
        self.pos_x = state[0]
        self.pos_y = state[1]
        self.velocity = state[2]
        self.yaw = state[3]
        self.yaw_rate = state[4]

    def __str__(self):
        text = f"Current State: \n  x: {self.pos_x}\n  y: {self.pos_y}\n  velocity: {self.velocity}\n  yaw: {self.yaw}\n  yaw_rate: {self.yaw_rate}\n"
        return text

def nextState(state : CtrvState, time_step):
    delta_yaw = state.yaw_rate * time_step;

    if (state.yaw_rate == 0):
        delta_pos_x = state.velocity * math.cos(state.yaw) * time_step;
        delta_pos_y = state.velocity * math.sin(state.yaw) * time_step;

    elif (state.velocity == 0):
        delta_pos_x = 0
        delta_pos_y = 0

    else:
        vel_over_yaw_rate = state.velocity / state.yaw_rate
        delta_pos_x = vel_over_yaw_rate * math.sin(state.yaw + delta_yaw) - math.sin(state.yaw)
        delta_pos_y = -(vel_over_yaw_rate * math.cos(state.yaw + delta_yaw) - math.cos(state.yaw))

    new_state = [state.pos_x + delta_pos_x, state.pos_y + delta_pos_y, state.velocity, state.yaw + delta_yaw, state.yaw_rate]
    return CtrvState(new_state)

# Test for CTRV model:
#  Test CTRV nextState function against pure rotation
# expected: 0, 0, 0, 0.5, 1
state = [0, 0, 0, 0, 1]
time_step = 0.5
n = CtrvState(state)
n = nextState(n, time_step)
print(f"Test 1.) {n}")

# Test for CTRV model:
# Test CTRV nextState function against pure translation
# expected: 0.5, 0, 1, 0, 0
state = [0, 0, 1, 0, 0]
time_step = 0.5
n = CtrvState(state)
n = nextState(n, time_step)
print(f"Test 2.) {n}")

# Test for CTRV model:
# Test CTRV nextState function against mixed rotation and translation
# expected: 0.479425539, 0.122417438, 1, 0.5, 1
state = [0, 0, 1, 0, 1]
time_step = 0.5
n = CtrvState(state)
n = nextState(n, time_step)
print(f"Test 3.) {n}")
