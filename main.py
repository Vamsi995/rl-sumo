# from simulation.simulator import Simulator
# from utils.params import SumoParams, NetParams, VehicleParams
#
# sim_params = SumoParams(sim_step=0.1, render=True, emission_path='debug')
# net_params = NetParams()
# vehicles = VehicleParams()
# vehicles.add("human",
#              acceleration_controller=None,
#              routing_controller=None,
#              num_vehicles=22)
# simulator = Simulator(sim_params, net_params, vehicles)
# traci_conn = simulator.start_simulation()
#
# traci_conn.vehicle.addFull(
#             "veh_id_0",
#             "routetop_0",
#             departPos="36",
#             departSpeed="0"
# )
# traci_conn.simulationStep()


from collections import OrderedDict

import numpy as np

sumoBinary = "sumo-gui"

sumoCmd = [sumoBinary, "-c", "rlsumo/simulator/sumo_config/cfg/ring_road_20230617-1140031686982203.6779535.sumo.cfg",
           "--step-length", "0.1"]
import traci


traci.start(sumoCmd)
step = 0


rts = {
    "top": ["top", "left", "bottom", "right"],
    "left": ["left", "bottom", "right", "top"],
    "bottom": ["bottom", "right", "top", "left"],
    "right": ["right", "top", "left", "bottom"]
}

track_len = 260

vehicle_gap = 5 * 22

rem = track_len - vehicle_gap

gap = rem / 22

curr_pos = 0
curr_edge = 'bottom'
edge_len = 260 / 4

edge_limits = []
edge_limits.append(['bottom', edge_len])
edge_limits.append(['right', 2 * edge_len])
edge_limits.append(['top', 3 * edge_len])
edge_limits.append(['left', 4 * edge_len])

prev_edge_len = 0
route_id = edge_limits[0][0]
veh_ids = []
for i in range(22):
    if curr_pos > edge_limits[0][1]:
        prev_edge_len = edge_limits[0][1]
        edge_limits.pop(0)
        route_id = edge_limits[0][0]

    if "veh_id_{pos}".format(pos=i) == "veh_id_0":
        traci.vehicle.addFull(
            "veh_id_{pos}".format(pos=i),
            "route{route_id}_0".format(route_id=route_id),
            typeID="automated",
            departPos=str(curr_pos - prev_edge_len),
            departSpeed="0"
        )
        traci.vehicle.setColor("veh_id_0", (255, 0, 0, 255))
    else:
        traci.vehicle.addFull(
            "veh_id_{pos}".format(pos=i),
            "route{route_id}_0".format(route_id=route_id),
            typeID="human",
            departPos=str(curr_pos - prev_edge_len),
            departSpeed="0"
        )
    veh_ids.append("veh_id_{pos}".format(pos=i))
    # traci.vehicle.setSpeed("veh_id_{pos}".format(pos=i), 0)
    curr_pos += gap + 5

# traci.vehicle.subscribe("veh_id_0",
#                         [tc.VAR_ROAD_ID, tc.VAR_LENGTH, tc.VAR_MINGAP, tc.VAR_DISTANCE, tc.VAR_LANEPOSITION])
traci.simulationStep()
# traci.vehicle.subscribeLeader("veh_id_0", 2000)
for veh in veh_ids:
    traci.vehicle.subscribeLeader(veh, 2000)


a = 1
b = 1.5
T = 1
s0 = 2
v0 = 30
delta = 4
mingap = 2.5


def follower_stopper(veh_id):
    curvatures = [1.5, 1.0, 0.5]
    initial_x = [4.5, 5.0, 6.0]
    desired_velocity = 4.8

    leader_id, head = traci.vehicle.getLeader(veh_id, 2000)
    leader_speed = traci.vehicle.getSpeed(leader_id)
    current_velocity = traci.vehicle.getSpeed(veh_id)
    delta_v = min(leader_speed - current_velocity, 0)
    s = head + mingap

    delta_x1 = initial_x[0] + (1 / (2 * curvatures[0])) * (delta_v ** 2)
    delta_x2 = initial_x[1] + (1 / (2 * curvatures[1])) * (delta_v ** 2)
    delta_x3 = initial_x[2] + (1 / (2 * curvatures[2])) * (delta_v ** 2)
    vel = min(max(leader_speed, 0), desired_velocity)
    v_cmd = 0
    if s <= delta_x1:
        v_cmd = 0
    elif s <= delta_x2:
        v_cmd = vel * ((s - delta_x1) / (delta_x2 - delta_x1))
    elif s <= delta_x3:
        v_cmd = vel + ((desired_velocity - current_velocity) * ((s - delta_x2) / (delta_x3 - delta_x2)))
    elif s > delta_x3:
        v_cmd = desired_velocity

    # if vehicle.edge_id[0] == ":":
    #     return None
    return (v_cmd - current_velocity) / 0.1


def idm_control(veh_id):
    v = traci.vehicle.getSpeed(veh_id)
    lead_id, h = traci.vehicle.getLeader(veh_id, 2000)
    h += mingap
    lead_vel = traci.vehicle.getSpeed(lead_id)

    # print("Curr - ", traci.vehicle.getRoadID(veh_id), "Next - ", traci.vehicle.getRoadID(lead_id), h)
    # in order to deal with ZeroDivisionError
    if abs(h) < 1e-3:
        h = 1e-3

    s_star = s0 + max(
        0, v * T + v * (v - lead_vel) /
           (2 * np.sqrt(a * b)))
    return a * (1 - (v / v0) ** delta - (s_star / h) ** 2) + np.random.normal(0, 0.1)


# test = traci.vehicle.getLeader("veh_id_0")
# print(test)

avg_vel = []
max_headway = []
vmt = []
std = []
while step < 6000:
    results = traci.vehicle.getSubscriptionResults("veh_id_0")
    # print(results)
    accel = []
    for veh_id in veh_ids:
        accel.append(traci.vehicle.getSpeed(veh_id))

    # print(accel)
    # for veh_id in veh_ids:
    #     if veh_id == "veh_id_0":
    #         if step > 3000:
    #             traci.vehicle.setAccel("veh_id_0", 1)
    #             accel.append(follower_stopper(veh_id))
    #             continue
    #     accel.append(idm_control(veh_id))
    # # print(accel)
    #
    # for i in range(len(veh_ids)):
    #     veh_id = veh_ids[i]
    #     acc = accel[i]
    #     v = max([traci.vehicle.getSpeed(veh_id) + acc * 0.1, 0])
    #     traci.vehicle.slowDown(veh_id, v, 1e-3)

    for veh_id in veh_ids:
        curr_route = traci.vehicle.getRoute(veh_id)
        edge = traci.vehicle.getRoadID(veh_id)
        if curr_route[-1] == edge:
            traci.vehicle.setRoute(veh_id, rts[edge])

    traci.simulationStep()

    vel = []
    h = []
    vmt = []
    for veh_id in veh_ids:
        vel.append(traci.vehicle.getSpeed(veh_id))
        _, headway = traci.vehicle.getLeader(veh_id, 2000)
        vmt.append(traci.vehicle.getDistance(veh_id))
        h.append(headway)

    mean_vel = np.mean(vel)
    sum_std = 0
    for v in vel:
        sum_std += (v - mean_vel) ** 2
    std.append(np.sqrt(sum_std/21))

    max_headway.append(max(h))
    avg_vel.append(np.mean(vel))
    step += 1

t = 0
for ind, stand in enumerate(std[3000:]):
    if stand <= 0.1:
        t = ind + 1
        break

print("Stabilizing time: ", t)
print("Average Vel: ", np.mean(avg_vel))
print("Maximum Headway: ", max(max_headway[3000+2700:]))
print("VMT: ", sum(vmt) * 0.000621371)
traci.close()
