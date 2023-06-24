import collections


class TraCIVehicle:

    def __init__(self, sim_params):
        self.__ids = []  # ids of all vehicles
        self.__human_ids = []  # ids of human-driven vehicles
        self.__controlled_ids = []  # ids of flow-controlled vehicles
        self.__controlled_lc_ids = []  # ids of flow lc-controlled vehicles
        self.__rl_ids = []  # ids of rl-controlled vehicles
        self.__observed_ids = []  # ids of the observed vehicles

        # vehicles: Key = Vehicle ID, Value = Dictionary describing the vehicle
        # Ordered dictionary used to keep neural net inputs in order
        self.__vehicles = collections.OrderedDict()

        # create a sumo_observations variable that will carry all information
        # on the state of the vehicles for a given time step
        self.__sumo_obs = {}

        # total number of vehicles in the network
        self.num_vehicles = 0
        # number of rl vehicles in the network
        self.num_rl_vehicles = 0
        # number of vehicles  loaded but not departed vehicles
        self.num_not_departed = 0

        # contains the parameters associated with each type of vehicle
        self.type_parameters = {}

        # contain the minGap attribute of each type of vehicle
        self.minGap = {}

        # list of vehicle ids located in each edge in the network
        self._ids_by_edge = dict()

        # number of vehicles that entered the network for every time-step
        self._num_departed = []
        self._departed_ids = 0

        # number of vehicles to exit the network for every time-step
        self._num_arrived = []
        self._arrived_ids = 0
        self._arrived_rl_ids = []

        # whether or not to automatically color vehicles
        try:
            self._color_by_speed = sim_params.color_by_speed
            self._force_color_update = sim_params.force_color_update
        except AttributeError:
            self._force_color_update = False

        # old speeds used to compute accelerations
        self.previous_speeds = {}


    def initialize(self, vehicles):
        """Initialize vehicle state information.

        This is responsible for collecting vehicle type information from the
        VehicleParams object and placing them within the Vehicles kernel.

        Parameters
        ----------
        vehicles : flow.core.params.VehicleParams
            initial vehicle parameter information, including the types of
            individual vehicles and their initial speeds
        """
        self.type_parameters = vehicles.type_parameters
        self.minGap = vehicles.minGap
        self.num_vehicles = 0
        self.num_rl_vehicles = 0
        self.num_not_departed = 0

        self.__vehicles.clear()
        for typ in vehicles.initial:
            for i in range(typ['num_vehicles']):
                veh_id = '{}_{}'.format(typ['veh_id'], i)
                self.__vehicles[veh_id] = dict()
                self.__vehicles[veh_id]['type'] = typ['veh_id']
                self.__vehicles[veh_id]['initial_speed'] = typ['initial_speed']
                self.num_vehicles += 1
                # if typ['acceleration_controller'][0] == RLController:
                #     self.num_rl_vehicles += 1

