
sumoBinary = "sumo-gui"

sumoCmd = [sumoBinary, "-c", "rlsumo/simulator/sumo_config/cfg/ring_road_20230617-1140031686982203.6779535.sumo.cfg",
           "--step-length", "0.1"]
import traci
import traci.constants as tc

traci.start(sumoCmd)
step = 0
traci.vehicle.addFull(
        "veh_id_0",
        "routebottom_0",
        typeID="human",
        departPos="0",
        departSpeed="0"
)
traci.vehicle.setSpeed("veh_id_0", 0)
print("Initial Acceleration", traci.vehicle.getAcceleration("veh_id_0"))
print("Initial Velocity", traci.vehicle.getSpeed("veh_id_0"))

traci.simulationStep()
traci.vehicle.setSpeed("veh_id_0", 0.5)
traci.simulationStep()


print("Distance", traci.vehicle.getDistance("veh_id_0"))
print("Acceleration", traci.vehicle.getAcceleration("veh_id_0"))
print("Velocity", traci.vehicle.getSpeed("veh_id_0"))

traci.simulationStep()

print("Distance", traci.vehicle.getDistance("veh_id_0"))
print("Acceleration", traci.vehicle.getAcceleration("veh_id_0"))
print("Velocity", traci.vehicle.getSpeed("veh_id_0"))


#
# traci.vehicle.slowDown("veh_id_0")
# test = traci.vehicle.getLeader("veh_id_0")
# print(test)
# print(test, traci.vehicle.get())