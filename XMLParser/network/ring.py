from numpy import pi, sin, cos, linspace
import time

from XMLParser.utils.params import TrafficLightParams, VehicleParams


class RingRoadNetwork:

    def __init__(self, name, network_params):
        self.net_params = network_params
        self.traffic_lights=TrafficLightParams()
        self.vehicles=VehicleParams()
        self.orig_name = name  # To avoid repeated concatenation upon reset
        self.name = name + time.strftime('_%Y%m%d-%H%M%S') + str(time.time())

        # self.vehicles = vehicles
        # self.initial_config = initial_config
        # self.traffic_lights = traffic_lights

        # specify routes vehicles can take
        self.routes = self.specify_routes()
        # specify the attributes of the nodes
        self.nodes = self.specify_nodes()
        # collect the attributes of each edge
        self.edges = self.specify_edges()
        # specify the types attributes (default is None)
        self.types = self.specify_types()
        # specify the connection attributes (default is None)
        # self.connections = self.specify_connections(net_params)

        self.edge_starts = self.specify_edge_starts()
        self.internal_edge_starts = self.specify_internal_edge_starts()

    def specify_nodes(self):
        """See parent class."""
        length = self.net_params.length
        r = length / (2 * pi)

        nodes = [{
            "id": "bottom",
            "x": 0,
            "y": -r
        }, {
            "id": "right",
            "x": r,
            "y": 0
        }, {
            "id": "top",
            "x": 0,
            "y": r
        }, {
            "id": "left",
            "x": -r,
            "y": 0
        }]

        return nodes

    def specify_edges(self):
        """See parent class."""
        length = self.net_params.length
        resolution = self.net_params.resolution
        r = length / (2 * pi)
        edge_len = length / 4.

        edges = [{
            "id":
                "bottom",
            "type":
                "edgeType",
            "from":
                "bottom",
            "to":
                "right",
            "length":
                edge_len,
            "shape":
                [
                    (r * cos(t), r * sin(t))
                    for t in linspace(-pi / 2, 0, resolution)
                ]
        }, {
            "id":
                "right",
            "type":
                "edgeType",
            "from":
                "right",
            "to":
                "top",
            "length":
                edge_len,
            "shape":
                [
                    (r * cos(t), r * sin(t))
                    for t in linspace(0, pi / 2, resolution)
                ]
        }, {
            "id":
                "top",
            "type":
                "edgeType",
            "from":
                "top",
            "to":
                "left",
            "length":
                edge_len,
            "shape":
                [
                    (r * cos(t), r * sin(t))
                    for t in linspace(pi / 2, pi, resolution)
                ]
        }, {
            "id":
                "left",
            "type":
                "edgeType",
            "from":
                "left",
            "to":
                "bottom",
            "length":
                edge_len,
            "shape":
                [
                    (r * cos(t), r * sin(t))
                    for t in linspace(pi, 3 * pi / 2, resolution)
                ]
        }]

        return edges

    def specify_types(self):
        lanes = self.net_params.lanes
        speed_limit = self.net_params.speed_limit

        types = [{
            "id": "edgeType",
            "numLanes": lanes,
            "speed": speed_limit
        }]

        return types

    def specify_routes(self):
        rts = {
            "top": ["top", "left", "bottom", "right"],
            "left": ["left", "bottom", "right", "top"],
            "bottom": ["bottom", "right", "top", "left"],
            "right": ["right", "top", "left", "bottom"]
        }

        return rts

    def specify_edge_starts(self):
        """See parent class."""
        ring_length = self.net_params.length
        junction_length = 0.1  # length of inter-edge junctions

        edge_starts = [("bottom", 0),
                       ("right", 0.25 * ring_length + junction_length),
                       ("top", 0.5 * ring_length + 2 * junction_length),
                       ("left", 0.75 * ring_length + 3 * junction_length)]

        return edge_starts

    def specify_internal_edge_starts(self):
        """See parent class."""
        ring_length = self.net_params.length
        junction_length = 0.1  # length of inter-edge junctions

        edge_starts = [(":right_0", 0.25 * ring_length),
                       (":top_0", 0.5 * ring_length + junction_length),
                       (":left_0", 0.75 * ring_length + 2 * junction_length),
                       (":bottom_0", ring_length + 3 * junction_length)]

        return edge_starts
