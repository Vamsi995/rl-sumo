from XMLParser.network.traci_network import TraCINetwork
from XMLParser.simulation.traci_simulator import TracCISimulation
from XMLParser.network.ring import RingRoadNetwork
from XMLParser.vehicle.traci_vehicle import TraCIVehicle


class Simulator:

    def __init__(self, sim_params, net_params, veh_params):
        self.sim_params = sim_params
        self.net_params = net_params
        self.veh_params = veh_params
        self.simulator = TracCISimulation()
        self.network = TraCINetwork()
        self.vehicles = TraCIVehicle(veh_params)

    def start_simulation(self):
        self.network.generate_network(RingRoadNetwork('ring_road', self.net_params))
        self.vehicles.initialize(self.veh_params)
        traci_conn = self.simulator.start_simulation(self.network, self.sim_params)
        return traci_conn
    def reset_simulation(self):
        pass

    def update(self):
        pass

    def close(self):
        pass
