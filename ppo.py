import gymnasium as gym
from ray.rllib.algorithms import Algorithm
from ray import tune
from ray.rllib.algorithms.dqn import dqn
from ray.rllib.algorithms.ppo import ppo

from rlsumo.envs.ringroad import RingRoad
from rlsumo.utils.params import Params, VehicleParams, SimulationParams, RLParams
from experiment import Experiment


def train():
    params = Params(VehicleParams(env_vehicles=21, rl_vehicles=1, rl_action_type="continuous", agent_type="rl"),
                    RLParams(),
                    SimulationParams(render=False))
    experiment = Experiment("PPO", 6000, params)
    experiment.train()


def evaluate():
    env_config = {
        "params": Params(VehicleParams(env_vehicles=21, rl_vehicles=1, rl_action_type="continuous", agent_type="rl"),
                         RLParams(),
                         SimulationParams(render=True))
    }
    tune.register_env("ringroad_v0", lambda env_config: RingRoad(env_config))
    # algo = Algorithm.from_checkpoint(
    #     "/home/vamsi/ray_results/DQN_2023-07-01_15-38-25/DQN_ringroad_v0_37da4_00000_0_2023-07-01_15-38-25/checkpoint_000012")


    algorithm = (
        ppo.PPOConfig().training(
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
    algo.restore("/home/vamsi/ray_results/PPO/PPO_ringroad_v0_dc6ba_00000_0_2023-07-22_18-52-52/checkpoint_000001")
    env = RingRoad(env_config)
    obs, info = env.reset()
    terminated = False
    truncated = False
    episode_reward = 0
    while not terminated and not truncated:
        action = algo.compute_single_action(obs)
        print(action, type(action))
        obs, reward, terminated, truncated, info = env.step(action)
        episode_reward += reward

    print("Episode Reward: {rew}".format(rew=episode_reward))


if __name__ == "__main__":
    evaluate()
