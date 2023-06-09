import os
import sys

import numpy as np
from gymnasium import Env, spaces
from gymnasium.wrappers import EnvCompatibility
from rlsumo.simulator.traci_simulator import SimulationKernel
from rlsumo.vehicle.VehicleKernel import VehicleKernel

if 'SUMO_HOME' in os.environ:
 tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
 sys.path.append(tools)

class RingRoad(Env):

    def __init__(self, config):
        # Todo: Initialize action spaces/observation spaces
        self.time_step = 0
        self.config = config
        self.params = config["params"]
        self.done = False
        self.action_space = spaces.Discrete(2)
        self.observation_space = spaces.Box(low=np.array([0, -1, 0]), high=np.array([1, 1, 1]))
        self.simulator_kernel = SimulationKernel(self.params.simulation_params)
        self.vehicle_kernel = VehicleKernel(self.params.vehicle_params)
        self.kernel_api = None

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)

        self.time_step = 0
        self.done = False
        self.vehicle_kernel.clear_state()
        self.kernel_api = self.simulator_kernel.reset()
        new_obs = self.vehicle_kernel.reset(self.kernel_api)
        return new_obs, {}

    def step(self, rl_actions):

        self.time_step += 1

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
            return new_obs, -50, done, True, {}

        # Todo: Calculate reward
        rew = self.compute_rewards()
        return new_obs, rew, done, False, {}

    def compute_rewards(self):
        return self.vehicle_kernel.get_mean_velocity() - abs(self.vehicle_kernel.get_rl_accel())

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
        self.vehicle_kernel.clear_state()
        self.simulator_kernel.close()
