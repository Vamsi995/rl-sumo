from XMLParser.simulation.simulator import Simulator
from XMLParser.utils.params import SumoParams, VehicleParams, NetParams

sim_params=SumoParams()
veh_params=VehicleParams()
net_params=NetParams(length=230)

simulation = Simulator(sim_params=sim_params, veh_params=veh_params, net_params=net_params)

simulation.start_simulation()