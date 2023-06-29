from collections import OrderedDict

import numpy as np
from traci import TraCIException, FatalTraCIError

from rlsumo.vehicle.controllers.controller import RLControl, IDMControl
from rlsumo.vehicle.controllers.failsafe import Failsafe
from rlsumo.vehicle.vehicle import Vehicle, AgentVehicle, EnvVehicle


class VehicleKernel:

    def __init__(self, vehicle_params):
        self.vehicle_params = vehicle_params
        self.kernel_api = None
        self.vehicles_dict = OrderedDict()
        self.rl_vehicles_dict = OrderedDict()
        self.failsafe = Failsafe()

    def reset(self, kernel_api):
        self.kernel_api = kernel_api
        self._initialize_state()
        self.kernel_api.simulationStep()
        return self.get_simulator_state()

    def clear_state(self):
        if self.kernel_api is None:
            return
        # unsubscribe vehicles
        self.unsubscribe_vehicles()
        self.remove_vehicles()

        # Remove vehicles
        self.vehicles_dict.clear()
        self.rl_vehicles_dict.clear()

    def _initialize_state(self):

        num_env_vehicles = self.vehicle_params.env_vehicles
        num_rl_vehicles = self.vehicle_params.rl_vehicles
        total_num = num_env_vehicles + num_rl_vehicles

        track_len = self.vehicle_params.track_length
        vehicle_cover = self.vehicle_params.length * total_num

        rem_gap = track_len - vehicle_cover
        gap_req = rem_gap / total_num

        # Create vehicles
        self._create_vehicles(gap_req, total_num, num_rl_vehicles)
        # Subscribe to all vehicles
        self._subscribe_vehicles()

    def get_simulator_state(self):
        new_obs = []

        for veh_id in self.vehicles_dict.keys():
            self.vehicles_dict[veh_id].get_state()

        for veh_id in self.rl_vehicles_dict.keys():
            new_obs.append(self.rl_vehicles_dict[veh_id].get_state())

        return np.array(new_obs).flatten()

    def update_routes(self):
        for veh_id in self.vehicles_dict.keys():
            self.vehicles_dict[veh_id].update_route()

        for veh_id in self.rl_vehicles_dict.keys():
            self.rl_vehicles_dict[veh_id].update_route()

    def _distribute_rl_vehicles(self, total_num, num_rl_vehicles):
        return list(np.random.randint(0, total_num, num_rl_vehicles))
        # return [5]

    def _create_vehicles(self, gap_req, total_num, num_rl_vehicles):
        absolute_gap = gap_req + self.vehicle_params.length
        edge_len = self.vehicle_params.edge_len
        edge_limits = [['bottom', edge_len], ['right', 2 * edge_len], ['top', 3 * edge_len], ['left', 4 * edge_len]]
        prev_edge_len = 0
        route_id = edge_limits[0][0]
        curr_pos = 0
        indexes = self._distribute_rl_vehicles(total_num, num_rl_vehicles)
        for i in range(total_num):
            if curr_pos > edge_limits[0][1]:
                prev_edge_len = edge_limits[0][1]
                edge_limits.pop(0)
                route_id = edge_limits[0][0]

            if i + 1 >= total_num:
                leader_veh_id = "veh_id_{pos}".format(pos=0)
            else:
                leader_veh_id = "veh_id_{pos}".format(pos=i + 1)

            veh_id = "veh_id_{pos}".format(pos=i)
            if i in indexes:
                self.rl_vehicles_dict[veh_id] = AgentVehicle(
                    veh_id,
                    "route{route_id}_0".format(route_id=route_id),
                    self.vehicle_params.automated_veh,
                    departPos=str(curr_pos - prev_edge_len),
                    departSpeed="0",
                    kernel_api=self.kernel_api,
                    agent_type=self.vehicle_params.agent_type,
                    controller=self.vehicle_params.controller,
                    routes=self.vehicle_params.rts,
                    lane_pos=self.vehicle_params.lane_positions,
                    failsafe=self.failsafe,
                    leader_id=leader_veh_id,
                    action_type=self.vehicle_params.rl_action_type
                )
            else:
                self.vehicles_dict[veh_id] = EnvVehicle(
                    veh_id,
                    "route{route_id}_0".format(route_id=route_id),
                    self.vehicle_params.human_veh,
                    departPos=str(curr_pos - prev_edge_len),
                    departSpeed="0",
                    kernel_api=self.kernel_api,
                    agent_type="idm",
                    controller=self.vehicle_params.controller,
                    routes=self.vehicle_params.rts,
                    lane_pos=self.vehicle_params.lane_positions,
                    failsafe=self.failsafe,
                    leader_id=leader_veh_id,
                    action_type=None
                )
            curr_pos += absolute_gap

    def _subscribe_vehicles(self):
        for veh_id in list(self.vehicles_dict.keys()) + list(self.rl_vehicles_dict.keys()):
            self.kernel_api.vehicle.subscribe(veh_id, self.vehicle_params.vehicle_subs)
            self.kernel_api.vehicle.subscribeLeader(veh_id, 2000)

    def unsubscribe_vehicles(self):
        for veh_id in list(self.vehicles_dict.keys()) + list(self.rl_vehicles_dict.keys()):
            try:
                self.kernel_api.vehicle.unsubscribe(veh_id)
            except (FatalTraCIError, TraCIException):
                pass

    def remove_vehicles(self):
        for veh_id in list(self.vehicles_dict.keys()) + list(self.rl_vehicles_dict.keys()):
            try:
                self.kernel_api.vehicle.remove(veh_id)
            except (FatalTraCIError, TraCIException):
                pass

    def calculate_new_accelerations(self, rl_actions, time_step):

        for veh_id in self.vehicles_dict.keys():
            self.vehicles_dict[veh_id].calculate_acceleration()

        # if time_step > 3000:
        for veh_id in self.rl_vehicles_dict.keys():
            self.rl_vehicles_dict[veh_id].calculate_acceleration(rl_actions, time_step)

    def update_new_velocities(self, timestep):
        for veh_id in self.vehicles_dict.keys():
            self.vehicles_dict[veh_id].update_velocity()

        # if timestep > 3000:
        for veh_id in self.rl_vehicles_dict.keys():
            self.rl_vehicles_dict[veh_id].update_velocity()

    def get_mean_velocity(self):
        vel = []
        for veh_id in self.vehicles_dict.keys():
            vel.append(self.vehicles_dict[veh_id].v)

        # if timestep > 3000:
        for veh_id in self.rl_vehicles_dict.keys():
            vel.append(self.rl_vehicles_dict[veh_id].v)

        return np.mean(vel)

    def get_rl_accel(self):
        accel = []
        for veh_id in self.rl_vehicles_dict.keys():
            accel.append(self.rl_vehicles_dict[veh_id].acc)

        return np.mean(accel)
