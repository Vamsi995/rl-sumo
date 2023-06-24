import numpy as np
from gym import Env, spaces
from numpy import float32

from rlsumo.simulator.traci_simulator import SimulationKernel
from rlsumo.vehicle.VehicleKernel import VehicleKernel


class RingRoad(Env):

    def __init__(self, params=None):
        # Todo: Initialize action spaces/observation spaces
        self.time_step = 0
        self.params = params
        self.done = False
        self.action_space = spaces.Discrete(2)
        self.observation_space = spaces.Box(low=np.array([0, 0, 0]), high=np.array([1, 1, 1]))
        self.simulator_kernel = SimulationKernel(self.params.simulation_params)
        self.vehicle_kernel = VehicleKernel(self.params.vehicle_params)

        self.kernel_api = None

    def reset(self, seed=None, options=None):
        self.time_step = 0
        self.done = False

        self.kernel_api = self.simulator_kernel.reset()
        self.vehicle_kernel.reset(self.kernel_api)
        return np.array([0.0, 0.0, 0.0], dtype=float32), {}

    def step(self, rl_actions):

        self.time_step += 1

        # Todo: Get Current State
        # self.vehicle_kernel.get_simulator_state()

        # Todo: Calculate Accelerations of all vehicles - env and rl
        self.vehicle_kernel.calculate_new_accelerations(rl_actions, self.time_step)

        # Todo: update env and rl vehicles velocity
        self.vehicle_kernel.update_new_velocities(timestep=self.time_step)

        # Todo: update routes
        self.vehicle_kernel.update_routes()

        # Todo: simulation step
        self.kernel_api.simulationStep()

        # Todo: Get New State
        new_obs = self.vehicle_kernel.get_simulator_state()
        # Todo: Check for done
        done = self.is_done()

        # Todo: Collision Detection - premature termination
        if self.check_collision():
            return new_obs, 0, True, done, {}

        # Todo: Calculate reward
        rew = self.compute_rewards()
        return new_obs, rew, False, done, {}

    def compute_rewards(self):
        return 0

    def is_done(self):

        if self.params.rl_params.env_horizon == self.time_step:
            return True
        else:
            return False

    def check_collision(self):
        return len(self.kernel_api.simulation.getCollisions()) != 0
    def render(self):
        # render sim

        pass

    def close(self):
        # close env
        self.simulator_kernel.close()
