import numpy as np


class IDMControl:

    def __init__(self,
                 v0=30,
                 T=1,
                 a=1,
                 b=1.5,
                 delta=4,
                 s0=2,
                 noise=0):
        """Instantiate an IDM controller."""
        self.v0 = v0
        self.T = T
        self.a = a
        self.b = b
        self.delta = delta
        self.s0 = s0
        self.noise = noise

    def get_accel(self, vehicle):
        """See parent class."""
        v = vehicle.v

        leader_vel = vehicle.leader_speed
        h = vehicle.headway

        # in order to deal with ZeroDivisionError
        if abs(h) < 1e-3:
            h = 1e-3

        s_star = self.s0 + max(
            0, v * self.T + v * (v - leader_vel) /
               (2 * np.sqrt(self.a * self.b)))

        return self.a * (1 - (v / self.v0) ** self.delta - (s_star / h) ** 2)


class FollowerStopper:

    def __init__(self, desired_velocity=4.8, initial_x=None, curvatures=None):
        if curvatures is None:
            self.curvatures = [1.5, 1.0, 0.5]
        if initial_x is None:
            self.initial_x = [4.5, 5.0, 6.0]

        self.desired_velocity = desired_velocity

    def get_accel(self, vehicle):

        v_lead = vehicle.leader_speed
        delta_v = min(v_lead - vehicle.v, 0)
        s = vehicle.headway
        current_velocity = vehicle.v

        delta_x1 = self.initial_x[0] + (1 / (2 * self.curvatures[0])) * (delta_v ** 2)
        delta_x2 = self.initial_x[1] + (1 / (2 * self.curvatures[1])) * (delta_v ** 2)
        delta_x3 = self.initial_x[2] + (1 / (2 * self.curvatures[2])) * (delta_v ** 2)
        v = min(max(v_lead, 0), self.desired_velocity)
        v_cmd = 0
        if s <= delta_x1:
            v_cmd = 0
        elif s <= delta_x2:
            v_cmd = v * ((s - delta_x1) / (delta_x2 - delta_x1))
        elif s <= delta_x3:
            v_cmd = v + ((self.desired_velocity - current_velocity) * ((s - delta_x2) / (delta_x3 - delta_x2)))
        elif s > delta_x3:
            v_cmd = self.desired_velocity

        if vehicle.edge_id[0] == ":":
            return None

        # print(s, delta_x1, delta_x2, delta_x3, v_cmd)

        return (v_cmd - current_velocity) / 0.1


class PISaturation:

    def __init__(self):
        self.v_history = []

        # other parameters
        self.gamma = 2
        self.g_l = 7
        self.g_u = 30
        self.v_catch = 1

        # values that are updated by using their old information
        self.alpha = 0
        self.beta = 1 - 0.5 * self.alpha
        self.U = 0
        self.v_target = 0
        self.v_cmd = 0

    def get_accel(self, vehicle):
        self.max_accel = vehicle.kernel_api.vehicle.getAccel(vehicle.veh_id)

        lead_vel = vehicle.leader_speed
        this_vel = vehicle.v

        dx = vehicle.headway
        dv = lead_vel - this_vel
        dx_s = max(2 * dv, 4)

        # update the AV's velocity history
        self.v_history.append(this_vel)

        if len(self.v_history) == int(38 / 0.1):
            del self.v_history[0]

        # update desired velocity values
        v_des = np.mean(self.v_history)
        v_target = v_des + self.v_catch \
                   * min(max((dx - self.g_l) / (self.g_u - self.g_l), 0), 1)

        # update the alpha and beta values
        alpha = min(max((dx - dx_s) / self.gamma, 0), 1)
        beta = 1 - 0.5 * alpha

        # compute desired velocity
        self.v_cmd = beta * (alpha * v_target + (1 - alpha) * lead_vel) \
                     + (1 - beta) * self.v_cmd

        # compute the acceleration
        accel = (self.v_cmd - this_vel) / 0.1

        return min(accel, self.max_accel)


class RLControl:
    def __init__(self):
        pass

    def get_accel(self, rl_actions=None):
        return rl_actions
