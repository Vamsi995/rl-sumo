import gymnasium as gym
import numpy as np
import traci

from rlsumo.envs.multiagent_ringroad import MultiAgentRingRoad
from rlsumo.metrics.metrics import Metrics
from rlsumo.utils.params import Params, VehicleParams, SimulationParams, RLParams

params = Params(VehicleParams(env_vehicles=20, rl_vehicles=2, rl_action_type="discrete", agent_type="idm"),
                RLParams(),
                SimulationParams(render=True))
env_config = {
    "params": params
}
vmt = []
fuel = []

for i in range(1):

    test = MultiAgentRingRoad(env_config)
    obs, info = test.reset()

    metrics = Metrics(test)
    s = 0

    while s <= 6000:
        obs, rew, term, done, info = test.step(action=None)
        if term:
            print("Terminating")
            break
        metrics.step()
        s += 1

    print("Episode no: {step}".format(step=i))
    t = 0
    for ind, std in enumerate(metrics.stabilizing_time[3000:]):
        if std <= 0.1:
            t = ind + 1
            break

    print("Stabilizing time: {t}".format(t=t))
    print("Maximum Gap: {gap}".format(gap=max(metrics.maximum_gap[t + 3000:])))
    print("Vehicle Miles Travelled: {vmt}".format(vmt=sum(metrics.vmt_dict.values())))
    print("Fuel Consumption: {fuel}".format(fuel=sum(metrics.fuel_dict.values()) * 1e-3))
    print("Economy: {eco}".format(eco=metrics.vmt_dict["veh_id_0"]/metrics.fuel_dict["veh_id_0"]))
    vmt.append(sum(metrics.vmt_dict.values()))
    fuel.append(sum(metrics.fuel_dict.values()) * 1e-3)
    test.close()
    metrics.plot()

print(np.mean(vmt), np.mean(fuel))
