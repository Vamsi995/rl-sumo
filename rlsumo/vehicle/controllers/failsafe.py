

class Failsafe:

    def __init__(self):
        self.display_warnings = True
        self.delay = 1.0
        self.max_accel = 1.0
        self.max_decel = 4.5

    def get_safe_action_instantaneous(self, action, vehicle):

        if action is None:
            return None

        this_vel = vehicle.v
        sim_step = 0.1
        next_vel = this_vel + action * sim_step
        h = vehicle.headway

        if next_vel > 0:
            # the second and third terms cover (conservatively) the extra
            # distance the vehicle will cover before it fully decelerates
            if h < sim_step * next_vel + this_vel * 1e-3 + \
                    0.5 * this_vel * sim_step:
                # if the vehicle will crash into the vehicle ahead of it in the
                # next time step (assuming the vehicle ahead of it is not
                # moving), then stop immediately
                if self.display_warnings:
                    print(
                        "=====================================\n"
                        "Vehicle {} is about to crash. Instantaneous acceleration "
                        "clipping applied.\n"
                        "=====================================".format(vehicle.veh_id))

                return -this_vel / sim_step
            else:
                # if the vehicle is not in danger of crashing, continue with
                # the requested action
                return action
        else:
            return action

    def get_safe_velocity_action(self, action, vehicle):

        if action is None:
            return None

        safe_velocity = self.safe_velocity(vehicle)
        this_vel = vehicle.v
        sim_step = 0.1

        if this_vel + action * sim_step > safe_velocity:
            if safe_velocity > 0:
                return (safe_velocity - this_vel) / sim_step
            else:
                return -this_vel / sim_step
        else:
            return action

    def safe_velocity(self, vehicle):

        lead_vel = vehicle.leader_speed
        this_vel = vehicle.v
        sim_step = 0.1
        h = vehicle.headway
        dv = lead_vel - this_vel

        v_safe = 2 * h / sim_step + dv - this_vel * (2 * self.delay)

        if this_vel > v_safe:
            if self.display_warnings:
                print(
                    "=====================================\n"
                    "Speed of vehicle {} is greater than safe speed. Safe velocity "
                    "clipping applied.\n"
                    "=====================================".format(vehicle.veh_id))

        return v_safe

    def get_obey_speed_limit_action(self, action, vehicle):

        if action is None:
            return None

        this_vel = vehicle.v
        sim_step = 0.1
        edge_speed_limit = 30
        if this_vel + action * sim_step > edge_speed_limit:
            if edge_speed_limit > 0:
                if self.display_warnings:
                    print(
                        "=====================================\n"
                        "Speed of vehicle {} is greater than speed limit. Obey "
                        "speed limit clipping applied.\n"
                        "=====================================".format(vehicle.veh_id))
                return (edge_speed_limit - this_vel) / sim_step
            else:
                return -this_vel / sim_step
        else:
            return action

    def get_feasible_action(self, action, vehicle):

        if action is None:
            return None

        if action > self.max_accel:
            action = self.max_accel

            if self.display_warnings:
                print(
                    "=====================================\n"
                    "Acceleration of vehicle {} is greater than the max "
                    "acceleration. Feasible acceleration clipping applied.\n"
                    "=====================================".format(vehicle.veh_id))

        if action < -self.max_decel:
            action = -self.max_decel

            if self.display_warnings:
                print(
                    "=====================================\n"
                    "Deceleration of vehicle {} is greater than the max "
                    "deceleration. Feasible acceleration clipping applied.\n"
                    "=====================================".format(vehicle.veh_id))

        return action
