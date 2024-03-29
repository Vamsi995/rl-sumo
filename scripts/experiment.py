from ray import tune, air
from rlsumo.envs.ringroad import RingRoad


class Experiment:

    def __init__(self, algorithm, time_steps, env_config, algorithm_config):
        self.env_config = env_config
        self.algorithm_config = algorithm_config
        self.results = None
        self.time_steps = time_steps
        self.algo = algorithm
        tune.register_env("ringroad_v0", lambda env_config: RingRoad(env_config))

    def train(self):
        import ray
        ray.init(ignore_reinit_error=True, num_cpus=5, num_gpus=1)
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
                name="DQN",
                storage_path="./results/",
                stop=stop,
                verbose=1,
                checkpoint_config=air.CheckpointConfig(
                    checkpoint_frequency=10, checkpoint_at_end=True
                ),
            ),
        ).fit()

        ray.shutdown()

    def evaluate(self, algo):
        # # Get the best result based on a particular metric.
        # best_result = self.results.get_best_result(metric="episode_reward_mean", mode="max")
        #
        # # Get the best checkpoint corresponding to the best result.
        # best_checkpoint = best_result.checkpoint
        #
        # algo = Algorithm.from_checkpoint(best_checkpoint)
        import ray
        ray.init(ignore_reinit_error=True, num_cpus=1, num_gpus=0)
        env = RingRoad(self.env_config)
        obs, info = env.reset()
        terminated = False
        truncated = False
        episode_reward = 0
        while not terminated and not truncated:
            action = algo.compute_single_action(obs)
            print(action)
            obs, reward, terminated, truncated, info = env.step(action)
            episode_reward += reward

        print("Episode Reward: {rew}".format(rew=episode_reward))

        ray.shutdown()