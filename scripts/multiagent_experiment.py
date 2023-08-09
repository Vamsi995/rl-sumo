import ray
from ray import tune, air
from ray.rllib.policy.policy import PolicySpec

from rlsumo.envs.multiagent_ringroad import MultiAgentRingRoad


class MultiAgentExperiment:

    def __init__(self, algorithm, time_steps, env_config, algorithm_config):
        self.time_steps = time_steps
        self.algo = algorithm
        self.env_config = env_config
        self.algorithm_config = algorithm_config
        self.results = None
        ray.shutdown()
        ray.init()
        tune.register_env("maringroad_v0", lambda env_config: MultiAgentRingRoad(env_config))

    def define_policies(self):
        env = MultiAgentRingRoad(self.env_config)
        agent_ids = env.create_agent_ids()
        policies = {}
        for ag in agent_ids:
            policies[ag] = PolicySpec()
        return policies

    @staticmethod
    def policy_map_fn(agent_id: str, _episode=None, _worker=None, **_kwargs) -> str:
        """
        Maps agent_id to policy_id
        """
        return agent_id

    def train(self):

        # Todo: Define algorithm
        algorithm = (
            self.algorithm_config
            .framework("torch")
            # Rollout
            .rollouts(
                batch_mode="complete_episodes",
                num_rollout_workers=3
            )
            .evaluation(evaluation_interval=10, evaluation_duration=1)
            # Resources
            .resources(num_gpus=1)
            .environment("maringroad_v0", env_config=self.env_config)
            .multi_agent(
                policies=self.define_policies(),
                policy_mapping_fn=lambda agent_id, episode, worker, **kwargs: agent_id
            )
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
        # best_result = self.results.get_best_result(metric="episode_reward_mean", mode="max")
        #
        # # Get the best checkpoint corresponding to the best result.
        # best_checkpoint = best_result.checkpoint
        #
        # algo = Algorithm.from_checkpoint(best_checkpoint)
        algorithm = (
            self.algorithm_config
            .framework("torch")
            # Rollout
            .rollouts(
                batch_mode="complete_episodes",
                num_rollout_workers=1
            )
            # Resources
            .resources(num_gpus=1)
            .environment("maringroad_v0", env_config=self.env_config)
            .multi_agent(
                policies=self.define_policies(),
                policy_mapping_fn=lambda agent_id, episode, worker, **kwargs: agent_id
            )
            # Reports
            # .reporting(min_time_s_per_iteration=5)
        )

        algo = algorithm.build()
        algo.restore(
            "/home/vamsi/ray_results/DQN/DQN_maringroad_v0_1c4ab_00000_0_2023-07-25_16-49-24/checkpoint_000001")

        env = MultiAgentRingRoad(self.env_config)
        obs, info = env.reset()
        terminated = {
            "__all__": False
        }
        truncated = {
            "__all__": False
        }
        episode_reward = 0

        while not terminated["__all__"] and not truncated["__all__"]:
            action = {}
            for agent_id, ob in obs.items():
                action[agent_id] = algo.compute_single_action(ob, policy_id=agent_id)
            print(action)
            obs, reward, terminated, truncated, info = env.step(action)
            episode_reward += sum(reward.values())

            print(terminated, truncated)

        print("Episode Reward: {rew}".format(rew=episode_reward))
