import os

import gymnasium as gym
import numpy as np
import ray
from ray import tune, air
from ray.rllib.algorithms import Algorithm
from ray.rllib.algorithms.dqn import dqn
from ray.rllib.algorithms.ppo import ppo
from ray.rllib.utils.replay_buffers import MultiAgentPrioritizedReplayBuffer
from rlsumo.envs.ringroad import RingRoad


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

        if self.algo == "DQN":
            return dqn.DQNConfig().training(
                lr=0.00001,
                gamma=0.99,
                td_error_loss_fn="huber",
                train_batch_size=128,
                model={
                    "fcnet_hiddens": [16, 16],
                    "fcnet_activation": "tanh"
                },
                replay_buffer_config={
                    "type": MultiAgentPrioritizedReplayBuffer,
                    "capacity": 50000
                },
                num_steps_sampled_before_learning_starts=1000,
                noisy=True,
                sigma0=0.28,
                double_q=True,
                dueling=True
            )
        # exploration_config = {
        #     "epsilon_timesteps": 10000,
        #     "final_epsilon": 0.01
        # },
        elif self.algo == "PPO":
            return ppo.PPOConfig().training(

                train_batch_size=6000,
                sgd_minibatch_size=512,
                num_sgd_iter=10,

                use_critic=True,
                use_gae=True,

                gamma=0.99,
                lambda_=0.95,
                lr=1e-4,

                model={
                    "fcnet_hiddens": [32, 32],
                    "fcnet_activation": "tanh",
                    "vf_share_layers": False
                },
                kl_coeff=0.3,
                kl_target=0.003,
                clip_param=0.2,
                vf_loss_coeff=0.6,
                grad_clip=0.5,
                grad_clip_by="global_norm"
                # shuffle_sequences=True,
                # entropy_coeff=0.001,
                # entropy_coeff_schedule=
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
                num_rollout_workers=3
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
            self.algo,
            param_space=algorithm.to_dict(),
            run_config=air.RunConfig(
                stop=stop,
                verbose=1,
                checkpoint_config=air.CheckpointConfig(
                    checkpoint_frequency=10, checkpoint_at_end=True
                ),
            ),
        ).fit()

        ray.shutdown()

    def evaluate(self):
        # # Get the best result based on a particular metric.
        best_result = self.results.get_best_result(metric="episode_reward_mean", mode="max")

        # Get the best checkpoint corresponding to the best result.
        best_checkpoint = best_result.checkpoint

        algo = Algorithm.from_checkpoint(best_checkpoint)

        env = gym.make('ringroad_v0', config=self.env_config)
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
