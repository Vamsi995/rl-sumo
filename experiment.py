import os

import gym
import numpy as np
import ray
from ray import tune, air
from ray.rllib.algorithms import Algorithm
from ray.rllib.algorithms.dqn import dqn
from ray.rllib.algorithms.ppo import ppo

from rlsumo.envs.ringroad import RingRoad
from rlsumo.utils.params import Params, SimulationParams, RLParams, VehicleParams


class Experiment:

    def __init__(self, algorithm, time_steps, params):
        self.env_config = None
        self.results = None
        self.time_steps = time_steps
        self.algo = algorithm
        self.params = params
        ray.shutdown()
        ray.init()
        tune.register_env("ringroad_v0", lambda env_config: RingRoad(env_config))

    def algorithm_choice(self):

        if self.algo == "dqn":
            return dqn.DQNConfig().training(
                lr=0.0001
            )
        elif self.algo == "ppo":
            return ppo.PPOConfig().training(
                gamma=0.99,
                lambda_=0.95,
                lr=0.001,
                sgd_minibatch_size=256,
                train_batch_size=4000,
            )

    def train(self):

        # Todo: Define env config
        self.env_config = {
            "params": self.params
        }

        # Todo: Define algorithm
        algorithm = (
            self.algorithm_choice()
            .framework("torch")
            # Rollout
            .rollouts(
                batch_mode="complete_episodes",
                num_rollout_workers=0
            )
            .evaluation(evaluation_interval=10, evaluation_duration=1)
            # Resources
            .resources(num_gpus=1)
            .environment("ringroad_v0", env_config=self.env_config)
            # Reports
            # .reporting(min_time_s_per_iteration=5)
        )

        # Todo: Stopping Condition
        stop = {
            "timesteps_total": self.time_steps
        }

        # Todo: Tune fit
        self.results = tune.Tuner(
            algorithm.algo_class,
            param_space=algorithm.to_dict(),
            run_config=air.RunConfig(
                stop=stop,
                verbose=1,
                checkpoint_config=air.CheckpointConfig(
                    checkpoint_frequency=10,
                ),
            ),
        ).fit()

        ray.shutdown()

    def evaluate(self):
        # # Get the best result based on a particular metric.
        # best_result = self.results.get_best_result(metric="episode_reward_mean", mode="max")
        #
        # # Get the best checkpoint corresponding to the best result.
        # best_checkpoint = best_result.checkpoint
        #
        # algo = Algorithm.from_checkpoint(best_checkpoint)

        env = gym.make('ringroad-v0', config=self.env_config)
        obs, info = env.reset()
        terminated = False
        truncated = False
        episode_reward = 0
        while not terminated and not truncated:
            # action = algo.compute_single_action(obs)
            action = np.random.randint(2)
            obs, reward, terminated, truncated, info = env.step(action)
            episode_reward += reward

        print("Episode Reward: {rew}".format(rew=episode_reward))
