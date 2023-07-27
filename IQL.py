import gymnasium as gym
from ray import tune
from ray.rllib.algorithms import Algorithm
from ray.rllib.algorithms.dqn import dqn
from ray.util.client import ray

from multiagent_experiment import MultiAgentExperiment
from rlsumo.envs.multiagent_ringroad import MultiAgentRingRoad
from rlsumo.envs.ringroad import RingRoad
from rlsumo.utils.params import Params, VehicleParams, SimulationParams, RLParams


def train():
    params = Params(VehicleParams(env_vehicles=20, rl_vehicles=2, rl_action_type="discrete", agent_type="rl"),
                    RLParams(),
                    SimulationParams(render=False))
    experiment = MultiAgentExperiment("DQN", 6000, params)
    experiment.train()


def evaluate():

    params = Params(VehicleParams(env_vehicles=20, rl_vehicles=2, rl_action_type="discrete", agent_type="rl"),
                         RLParams(),
                         SimulationParams(render=True))
    experiment = MultiAgentExperiment("DQN", 6000, params)
    experiment.evaluate()


if __name__ == "__main__":
    evaluate()
