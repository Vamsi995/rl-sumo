import numpy as np
from gymnasium import Env, spaces
from rlsumo.simulator.traci_simulator import SimulationKernel
from rlsumo.vehicle.VehicleKernel import VehicleKernel


class RingRoad(Env):

    def __init__(self, config):
        # Todo: Initialize action spaces/observation spaces
        self.time_step = 0
        self.config = config
        self.params = config["params"]
        self.done = False

        if self.params.vehicle_params.rl_action_type == "continuous":
            self.action_space = spaces.Box(low=np.array([-2]), high=np.array([1]), dtype=np.float64)
        else:
            self.action_space = spaces.Discrete(3)
        self.observation_space = spaces.Box(low=np.array([0.0, -1.0, 0], dtype=np.float32),
                                            high=np.array([1.0, 1.0, 1.0], dtype=np.float32),
                                            dtype=np.float64)
        self.simulator_kernel = SimulationKernel(self.params.simulation_params)
        self.vehicle_kernel = VehicleKernel(self.params.vehicle_params)
        self.kernel_api = None
        self.warmup = False

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)

        self.time_step = 0
        self.done = False
        self.vehicle_kernel.clear_state()
        self.kernel_api = self.simulator_kernel.reset()
        self.vehicle_kernel.reset(self.kernel_api)

        # Add warmup steps
        self.warmup_steps()
        new_obs = self.vehicle_kernel.get_simulator_state()
        self.time_step = 0
        self.done = False
        return new_obs, {}

    def warmup_steps(self):
        self.warmup = True
        for i in range(self.params.rl_params.warmup_steps):
            self.step(None)
        self.warmup = False

    def step(self, rl_actions):

        self.time_step += 1

        # Todo: Calculate Accelerations of all vehicles - env and rl
        self.vehicle_kernel.calculate_new_accelerations(rl_actions, self.time_step, self.warmup)

        # Todo: update env and rl vehicles velocity
        self.vehicle_kernel.update_new_velocities()

        # Todo: update routes
        self.vehicle_kernel.update_routes()

        # Todo: simulation step
        self.kernel_api.simulationStep()

        # Todo: Get New State
        new_obs = self.vehicle_kernel.get_simulator_state()

        # Todo: Check for done
        self.done = self.is_done()

        # Todo: Collision Detection - premature termination
        if self.check_collision():
            return new_obs, -50, self.done, True, {}

        # Todo: Calculate reward
        rew = self.compute_rewards()
        return new_obs, rew, self.done, False, {}

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
