from collections import OrderedDict

import numpy as np
from matplotlib import pyplot as plt


class Metrics:

    # Todo: Add tracking - average stabilizing time, maximum gap, vehicle miles travelled, fuel economy
    def __init__(self, env):

        self.env = env
        self.env_vehicles = env.vehicle_kernel.vehicles_dict
        self.rl_vehicles = env.vehicle_kernel.rl_vehicles_dict

        self.v_dict = dict()
        self.position_dict = dict()
        self.maximum_gap = []
        self.stabilizing_time = []
        self.vmt_dict = dict()
        self.fuel_dict = dict()
        self.acc_store = []
        self.headway_dict = dict()

        self.register()

    def get_maximum_headway(self):
        headways = []
        for veh_id, vehicle in self.env_vehicles.items():
            headways.append(vehicle.headway)

        for veh_id, vehicle in self.rl_vehicles.items():
            headways.append(vehicle.headway)

        return max(headways)

    def register(self):
        for veh_id, vehicle in self.env_vehicles.items():
            self.v_dict[veh_id] = [vehicle.v]
            self.position_dict[veh_id] = [vehicle.position]
            self.headway_dict[veh_id] = [vehicle.headway]

        for veh_id, vehicle in self.rl_vehicles.items():
            self.v_dict[veh_id] = [vehicle.v]
            self.position_dict[veh_id] = [vehicle.position]
            self.headway_dict[veh_id] = [vehicle.headway]

        self.store_maximum_gap()

    def store_v(self):
        for veh_id, vehicle in self.env_vehicles.items():
            self.v_dict[veh_id].append(vehicle.v)

        for veh_id, vehicle in self.rl_vehicles.items():
            self.v_dict[veh_id].append(vehicle.v)

    def store_xy(self):
        for veh_id, vehicle in self.env_vehicles.items():
            self.position_dict[veh_id].append(vehicle.position)

        for veh_id, vehicle in self.rl_vehicles.items():
            self.position_dict[veh_id].append(vehicle.position)

    def step(self):
        self.store_v()
        self.store_xy()
        self.calculate_stabilizing_time()
        self.store_maximum_gap()
        self.store_vehicle_miles_travelled()
        self.store_fuel_consumption()
        self.store_acc()
        self.store_headway()

    def plot_position(self):
        plt.figure(figsize=(15, 5))
        global s
        for veh_id, vehicle in list(self.env_vehicles.items()) + list(self.rl_vehicles.items()):
            y = self.position_dict[veh_id]
            v = self.v_dict[veh_id]
            x = np.arange(0, len(y)) * 0.1
            s = plt.scatter(x, y, c=v, cmap=plt.get_cmap("viridis"), marker='.')

        plt.colorbar(s, label="Velocity (m/s)")
        plt.xlabel("Time (s)")
        plt.ylabel("Position (m)")
        plt.show()

    def plot_acc(self):

        y = self.acc_store
        x = np.arange(0, len(y)) * 0.1
        plt.plot(x, y, color='r')
        plt.xlabel("Time (s)")
        plt.ylabel("Acc (m/s)")
        plt.show()

    def plot_velocities(self):
        plt.figure(figsize=(15, 5))
        for veh_id in self.env_vehicles.keys():
            y = self.v_dict[veh_id]
            x = np.arange(0, len(y)) * 0.1
            plt.plot(x, y, color='gray')
        for ag_id in self.rl_vehicles.keys():
            y = self.v_dict[ag_id]
            x = np.arange(0, len(y)) * 0.1
            plt.plot(x, y, color='r')
        plt.xlabel("Time (s)")
        plt.ylabel("Velocity (m/s)")
        plt.show()

    def plot_headway(self):
        plt.figure(figsize=(15, 5))
        for veh_id in self.env_vehicles.keys():
            y = self.headway_dict[veh_id]
            x = np.arange(0, len(y)) * 0.1
            plt.plot(x, y, color='gray')
        for ag_id in self.rl_vehicles.keys():
            y = self.headway_dict[ag_id]
            x = np.arange(0, len(y)) * 0.1
            plt.plot(x, y, color='r')
        plt.xlabel("Time (s)")
        plt.ylabel("Headway (m/s)")
        plt.show()

    def plot(self):
        self.plot_velocities()
        self.plot_headway()
        self.plot_acc()
        # self.plot_position()

    def calculate_stabilizing_time(self):
        velocities = []
        for veh_id, vehicle in list(self.env_vehicles.items()) + list(self.rl_vehicles.items()):
            velocities.append(vehicle.v)

        vel_mean = np.mean(velocities)
        sum_std = 0
        for vel in velocities:
            sum_std += (vel - vel_mean) ** 2

        std = np.sqrt(sum_std / 21)
        self.stabilizing_time.append(std)
        # print("TimeStep: {time}, Stabilizing Time: {s}".format(time=self.env.time_step, s=std))

    def store_maximum_gap(self):
        self.maximum_gap.append(self.get_maximum_headway())

    def store_vehicle_miles_travelled(self):
        for veh_id, vehicle in self.env_vehicles.items():
            self.vmt_dict[veh_id] = vehicle.distance * 0.000621371

        for veh_id, vehicle in self.rl_vehicles.items():
            self.vmt_dict[veh_id] = vehicle.distance * 0.000621371

    def store_fuel_consumption(self):
        for veh_id, vehicle in self.env_vehicles.items():
            self.fuel_dict[veh_id] = vehicle.fuel_consumption * 0.264172 + self.fuel_dict.get(veh_id, 0)

        for veh_id, vehicle in self.rl_vehicles.items():
            self.fuel_dict[veh_id] = vehicle.fuel_consumption * 0.264172 + self.fuel_dict.get(veh_id, 0)

    def store_acc(self):

        for veh_id, vehicle in self.rl_vehicles.items():
            self.acc_store.append(vehicle.acc)

    def store_headway(self):
        for veh_id, vehicle in self.env_vehicles.items():
            self.headway_dict[veh_id].append(vehicle.headway)

        for veh_id, vehicle in self.rl_vehicles.items():
            self.headway_dict[veh_id].append(vehicle.headway)

