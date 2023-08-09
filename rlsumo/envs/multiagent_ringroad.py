import numpy as np
from ray.rllib import MultiAgentEnv
from gymnasium import spaces

from rlsumo.simulator.traci_simulator import SimulationKernel
from rlsumo.vehicle.VehicleKernel import VehicleKernel


class MultiAgentRingRoad(MultiAgentEnv):

    def __init__(self, config):
        super().__init__()
        self.time_step = 0
        self.config = config
        self.params = config["params"]
        self._agent_ids = self.create_agent_ids()

        self.dones = {}
        self.terminated = {}

        if self.params.vehicle_params.rl_action_type == "continuous":
            self.action_measure = spaces.Box(low=np.array([-2]), high=np.array([1]), dtype=np.float64)
        else:
            self.action_measure = spaces.Discrete(3)

        self.action_space = spaces.Dict({ag: self.action_measure for ag in self._agent_ids})
        self.observation_space = spaces.Dict({ag: spaces.Box(low=np.array([0.0, -1.0, 0]),
                                                             high=np.array([1.0, 1.0, 1.0]),
                                                             shape=(3,),
                                                             dtype=np.float64) for ag in self._agent_ids})

        self.simulator_kernel = SimulationKernel(self.params.simulation_params)
        self.vehicle_kernel = VehicleKernel(self.params.vehicle_params)
        self.kernel_api = None
        self.warmup = False

    def create_agent_ids(self):
        _agent_ids = []
        for i in range(self.params.vehicle_params.rl_vehicles):
            _agent_ids.append(f"agent{i}")
        return set(_agent_ids)

    def reset_env_variables(self):
        self.time_step = 0
        self.dones = {ag: False for ag in self.vehicle_kernel.rl_vehicles_dict.keys()}
        self.dones["__all__"] = False
        self.terminated = {ag: False for ag in self.vehicle_kernel.rl_vehicles_dict.keys()}
        self.terminated["__all__"] = False

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)

        self.reset_env_variables()
        self.vehicle_kernel.clear_state()
        self.kernel_api, track_len = self.simulator_kernel.reset()
        self.vehicle_kernel.ma_reset(kernel_api=self.kernel_api, track_len=track_len)

        self.warmup_steps()
        new_obs = self.vehicle_kernel.get_ma_simulator_state()
        self.reset_env_variables()
        return new_obs, {}

    def warmup_steps(self):
        self.warmup = True
        for i in range(self.params.rl_params.warmup_steps):
            self.step({ag: None for ag in self.vehicle_kernel.rl_vehicles_dict.keys()})
        self.warmup = False

    def step(self, action_dict):

        self.time_step += 1
        self.vehicle_kernel.calculate_new_ma_accelerations(action_dict, self.time_step, self.warmup)

        self.vehicle_kernel.update_new_velocities()
        self.vehicle_kernel.update_routes()

        self.kernel_api.simulationStep()

        new_obs = self.vehicle_kernel.get_ma_simulator_state()

        self.dones = self.is_done()
        self.terminated = self.is_terminated()

        rew = self.compute_rewards(action_dict)
        return new_obs, rew, self.dones, self.is_terminated(), {}

    def compute_rewards(self, action):
        if self.terminated["__all__"]:
            # Todo: Give Appropriate rewards based on the vehicle collided.
            return {ag: -50 for ag in action.keys()}

        return {ag: self.vehicle_kernel.get_mean_velocity() - self.vehicle_kernel.rl_vehicles_dict[ag].acc for ag in
                action.keys()}

    def is_done(self):
        dones = {}
        if self.params.rl_params.env_horizon == self.time_step:
            dones = {ag: True for ag in self.vehicle_kernel.rl_vehicles_dict.keys()}
            dones["__all__"] = True
        else:
            dones = {ag: False for ag in self.vehicle_kernel.rl_vehicles_dict.keys()}
            dones["__all__"] = False
        return dones

    def is_terminated(self):
        terminated = {}
        if self.check_collision():
            terminated = {ag: True for ag in self.vehicle_kernel.rl_vehicles_dict.keys()}
            terminated["__all__"] = True
        else:
            terminated = {ag: False for ag in self.vehicle_kernel.rl_vehicles_dict.keys()}
            terminated["__all__"] = False
        return terminated

    def check_collision(self):
        return len(self.kernel_api.simulation.getCollisions()) != 0

    def render(self) -> None:
        pass

    def close(self):
        pass
