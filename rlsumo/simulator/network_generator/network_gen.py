import numpy as np

from rlsumo.simulator.network_generator.ring import RingRoadNetwork
from rlsumo.simulator.network_generator.traci_network import TraCINetwork
from rlsumo.utils.params import NetParams


class NetworkGenerator:

    def __init__(self):
        self.network = TraCINetwork()
        self.length = None

    def generate_network(self):
        self.length = np.random.randint(230, 271)
        self.network.generate_network(RingRoadNetwork('ring_road', NetParams(length=self.length)))

    def get_length(self):
        return self.network.network.length

    def close(self):
        self.network.close()
