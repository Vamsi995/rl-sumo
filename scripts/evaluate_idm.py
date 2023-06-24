import gym

from rlsumo.utils.params import Params, VehicleParams, RLParams, SimulationParams


def main():
    params = Params(VehicleParams(), RLParams(), SimulationParams(render=True))
    env = gym.make('ringroad-v0', params=params)

    obs = env.reset()






if __name__ == "__main__":
    main()