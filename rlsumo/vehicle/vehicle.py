import numpy as np
import traci.constants as tc


class Vehicle:

    def __init__(self, veh_id, route_id, type_id, departPos, departSpeed, kernel_api, agent_type, controller, routes,
                 lane_pos, failsafe, leader_id):
        self.veh_id = veh_id
        self.route_id = route_id
        self.type_id = type_id
        self.departPos = departPos
        self.initial_vel = departSpeed
        self.kernel_api = kernel_api
        self.agent_type = agent_type
        self.controller = controller
        self.failsafe = failsafe
        self.leader_id = leader_id

        self.v = None
        self.acc = None
        self.distance = None
        self.headway = None
        self.fuel_consumption = None
        self.edge_id = None
        self.min_gap = 2.0
        self.leader_speed = None
        self.route = None
        self.position = None

        self.rts = routes
        self.lane_pos = lane_pos
        self.add()

    def add(self):
        self.kernel_api.vehicle.addFull(
            vehID=self.veh_id,
            routeID=self.route_id,
            typeID=self.type_id,
            departPos=self.departPos,
            departSpeed=self.initial_vel,
        )
        self.kernel_api.vehicle.setSpeed(self.veh_id, 0)
        self.kernel_api.vehicle.setSpeedMode(self.veh_id, 31)

    def get_state(self):
        result = self.kernel_api.vehicle.getSubscriptionResults(self.veh_id)
        self.v = result[tc.VAR_SPEED]
        self.acc = result[tc.VAR_ACCELERATION]
        self.distance = result[tc.VAR_DISTANCE]
        self.fuel_consumption = result[tc.VAR_FUELCONSUMPTION] * 0.1
        self.edge_id = self.kernel_api.vehicle.getRoadID(self.veh_id)
        # self.leader_id, self.headway = self.kernel_api.vehicle.getLeader(self.veh_id, 2000)
        # self.headway += self.min_gap
        self.route = self.kernel_api.vehicle.getRoute(self.veh_id)
        self.leader_speed = self.kernel_api.vehicle.getSpeed(self.leader_id)
        self.position = result[tc.VAR_LANEPOSITION] + self.lane_pos[self.edge_id]

        leader_edge = self.kernel_api.vehicle.getRoadID(self.leader_id)
        leader_pos = self.kernel_api.vehicle.getLanePosition(self.leader_id) + self.lane_pos[leader_edge]

        if leader_pos > self.position:
            self.headway = leader_pos - self.position - 5
        else:
            self.headway = leader_pos + (260 - self.position) - 5

    def update_velocity(self):
        if self.acc is not None:
            self.v = max(self.v + self.acc * 0.1, 0)
            self.kernel_api.vehicle.slowDown(self.veh_id, self.v, 1e-3)

    def update_route(self):
        if self.route[-1] == self.edge_id:
            self.kernel_api.vehicle.setRoute(self.veh_id, self.rts[self.edge_id])


class EnvVehicle(Vehicle):

    def __init__(self, veh_id, route_id, type_id, departPos, departSpeed, kernel_api, agent_type, controller, routes,
                 lane_pos, failsafe, leader_id):
        super(EnvVehicle, self).__init__(veh_id, route_id, type_id, departPos, departSpeed, kernel_api, agent_type,
                                         controller,
                                         routes, lane_pos, failsafe, leader_id)

    def calculate_acceleration(self):
        self.acc = self.controller[self.agent_type].get_accel(self) + np.random.normal(0, 0.1)


class AgentVehicle(Vehicle):

    def __init__(self, veh_id, route_id, type_id, departPos, departSpeed, kernel_api, agent_type, controller, routes,
                 lane_pos, failsafe, leader_id):
        super(AgentVehicle, self).__init__(veh_id, route_id, type_id, departPos, departSpeed, kernel_api, agent_type,
                                           controller,
                                           routes,
                                           lane_pos, failsafe, leader_id)

        self.kernel_api.vehicle.setColor(self.veh_id, (255, 0, 0, 255))

    def calculate_acceleration(self, time_step):
        if time_step > 3000:
            # Todo: Desired Controller
            # self.kernel_api.vehicle.setAccel(self.veh_id, 0.5)
            # self.kernel_api.vehicle.setDecel(self.veh_id, 1.5)
            self.acc = self.controller[self.agent_type].get_accel(self)
            # self.acc = self.failsafe.get_feasible_action(self.acc, self)
            # self.acc = self.failsafe.get_safe_action_instantaneous(self.acc, self)
            # self.acc = self.failsafe.get_safe_velocity_action(self.acc, self)
            # self.acc = self.failsafe.get_obey_speed_limit_action(self.acc, self)
        else:
            # Todo: IDM Control
            self.acc = self.controller['idm'].get_accel(self)

    def get_state(self):
        super().get_state()

        return [self.v, self.headway]

