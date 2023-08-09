import gym

from rlsumo.envs.ringroad import RingRoad
from rlsumo.utils.params import Params, VehicleParams, RLParams, SimulationParams


def main():
    params = Params(VehicleParams(), RLParams(), SimulationParams(render=True))
    env_config = {
        "params": params
    }
    env = RingRoad(env_config)

    obs = env.reset()






if __name__ == "__main__":
    main()