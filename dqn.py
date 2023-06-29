from rlsumo.utils.params import Params, VehicleParams, SimulationParams, RLParams
from experiment import Experiment


def main():
    params = Params(VehicleParams(env_vehicles=21, rl_vehicles=1, rl_action_type="discrete", agent_type="rl"),
                    RLParams(),
                    SimulationParams(render=False))
    experiment = Experiment("dqn", 6000, params)
    experiment.train()


if __name__ == "__main__":
    main()
