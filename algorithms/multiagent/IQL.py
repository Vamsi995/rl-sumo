from ray.rllib.algorithms.dqn import dqn
from ray.rllib.utils.replay_buffers import MultiAgentPrioritizedReplayBuffer

from rlsumo.utils.params import Params, VehicleParams, SimulationParams, RLParams
from scripts.multiagent_experiment import MultiAgentExperiment

env_config = {
    "params": Params(VehicleParams(env_vehicles=20, rl_vehicles=2, rl_action_type="discrete", agent_type="rl"),
                     RLParams(),
                     SimulationParams(render=False))
}

algorithm_config = dqn.DQNConfig().training(
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


def train():
    experiment = MultiAgentExperiment("DQN", 6000, env_config, algorithm_config)
    experiment.train()


def evaluate():
    env_config["params"].simulation_params = SimulationParams(render=True)
    experiment = MultiAgentExperiment("DQN", 6000, env_config, algorithm_config)
    experiment.evaluate()


if __name__ == "__main__":
    evaluate()
