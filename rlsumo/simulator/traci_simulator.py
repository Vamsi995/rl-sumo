import os
import signal
import subprocess
import time
import traceback

import numpy as np
import traci

# Number of retries on restarting SUMO before giving up
RETRIES_ON_ERROR = 10
SUMO_SLEEP = 1.0


class SimulationKernel:

    def __init__(self, simulation_params):
        self.sumo_proc = None
        self.simulation_params = simulation_params
        self.traci_connection = None

    def simulation_step(self):
        if self.traci_connection is not None:
            self.traci_connection.simulationStep()

    def start_simulation(self):
        error = None
        for _ in range(RETRIES_ON_ERROR):
            try:
                # port number the sumo instance will be run on
                port = traci.getFreeSocketPort()

                sumo_binary = "sumo-gui" if self.simulation_params.render is True \
                    else "sumo"

                # command used to start sumo
                sumo_call = [
                    sumo_binary, "-c", self.simulation_params.sim_config_path,
                    "--remote-port", str(port),
                    "--step-length", str(self.simulation_params.sim_step)
                ]

                # use a ballistic integration step (if request)
                if self.simulation_params.use_ballistic:
                    sumo_call.append("--step-method.ballistic")

                # ignore step logs (if requested)
                if self.simulation_params.no_step_log:
                    sumo_call.append("--no-step-log")

                # add the lateral resolution of the sublanes (if requested)
                if self.simulation_params.lateral_resolution is not None:
                    sumo_call.append("--lateral-resolution")
                    sumo_call.append(str(self.simulation_params.lateral_resolution))

                if self.simulation_params.overtake_right:
                    sumo_call.append("--lanechange.overtake-right")
                    sumo_call.append("true")

                # specify a simulation seed (if requested)
                if self.simulation_params.seed is not None:
                    sumo_call.append("--seed")
                    sumo_call.append(str(self.simulation_params.seed))

                # if not self.simulation_params.print_warnings:
                #     sumo_call.append("--no-warnings")
                #     sumo_call.append("true")

                # set the time it takes for a gridlock teleport to occur
                sumo_call.append("--time-to-teleport")
                sumo_call.append(str(int(self.simulation_params.teleport_time)))

                sumo_call.append("--collision.action")
                sumo_call.append("warn")

                # check collisions at intersections
                sumo_call.append("--collision.check-junctions")
                sumo_call.append("true")

                sumo_call.append("--emissions.volumetric-fuel")
                sumo_call.append("true")

                # Opening the I/O thread to SUMO
                self.sumo_proc = subprocess.Popen(
                    sumo_call,
                    stdout=subprocess.DEVNULL
                )

                # wait a small period of time for the subprocess to activate
                # before trying to connect with traci
                if os.environ.get("TEST_FLAG", 0):
                    time.sleep(0.1)
                else:
                    time.sleep(SUMO_SLEEP)

                traci_connection = traci.connect(port)
                # traci_connection.setOrder(0)
                self.traci_connection = traci_connection
                traci_connection.simulationStep()

                return traci_connection
            except Exception as e:
                print("Error during start: {}".format(traceback.format_exc()))
                error = e
                self.teardown_sumo()
        raise error

    def reset(self):
        self.close()
        return self.start_simulation()

    def teardown_sumo(self):
        """Kill the sumo subprocess instance."""
        try:
            if self.sumo_proc is not None:
                self.sumo_proc.kill()
                os.killpg(self.sumo_proc.pid, signal.SIGTERM)
        except Exception as e:
            print("Error during teardown: {}".format(e))

    def close(self):
        if self.traci_connection is not None:
            self.traci_connection.close()
        # if self.sumo_proc is not None:
        #     self.teardown_sumo()
