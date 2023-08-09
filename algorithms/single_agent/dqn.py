from ray import tune
from ray.rllib.algorithms.dqn import dqn
from ray.rllib.utils.replay_buffers import MultiAgentPrioritizedReplayBuffer

from rlsumo.envs.ringroad import RingRoad
from rlsumo.utils.params import Params, VehicleParams, SimulationParams, RLParams
from scripts.experiment import Experiment

env_config = {
    "params": Params(VehicleParams(env_vehicles=21, rl_vehicles=1, rl_action_type="discrete", agent_type="rl"),
                     RLParams(),
                     SimulationParams(render=False))
}

algorithm_config = dqn.DQNConfig().training(
    lr=0.00001,
    gamma=0.99,
    td_error_loss_fn="huber",
    train_batch_size=32,
    model={
        "fcnet_hiddens": [16, 16],
        "fcnet_activation": "tanh"
    },
    replay_buffer_config={
        "type": "MultiAgentPrioritizedReplayBuffer",
        "capacity": 50000
    },
    num_steps_sampled_before_learning_starts=5000,
    # noisy=True,
    # sigma0=0.28,
    # double_q=True,
    # dueling=True
)


# exploration_config = {
#     "epsilon_timesteps": 10000,
#     "final_epsilon": 0.01
# },


def train():
    experiment = Experiment("DQN", 30000, env_config, algorithm_config)
    experiment.train()


def evaluate():
    env_config["params"].simulation_params = SimulationParams(render=True)
    tune.register_env("ringroad_v0", lambda env_config: RingRoad(env_config))
    # algo = Algorithm.from_checkpoint(
    #     "/home/vamsi/ray_results/DQN_2023-07-01_15-38-25/DQN_ringroad_v0_37da4_00000_0_2023-07-01_15-38-25/checkpoint_000012")

    algorithm = (
        dqn.DQNConfig().training(
            lr=0.0001
        )
        .framework("torch")
        # Rollout
        .rollouts(
            batch_mode="complete_episodes",
            num_rollout_workers=1
        )
        # .evaluation(evaluation_interval=10, evaluation_duration=1)
        # Resources
        .resources(num_gpus=1)
        .environment("ringroad_v0", env_config=env_config, disable_env_checking=True)
        # Reports
        # .reporting(min_time_s_per_iteration=5)
    )
    algo = algorithm.build()
    algo.restore(
        "/home/vamsi/ray_results/DQN_2023-07-01_15-38-25/DQN_ringroad_v0_37da4_00000_0_2023-07-01_15-38-25/checkpoint_000012")
    env = RingRoad(env_config)
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
