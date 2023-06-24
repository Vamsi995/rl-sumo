import logging
import os
import signal
import subprocess
import time
import traceback

import traci

from XMLParser.utils.utils import ensure_dir

# Number of retries on restarting SUMO before giving up
RETRIES_ON_ERROR = 10
SUMO_SLEEP = 1.0


class TracCISimulation:

    def __init__(self):
        pass

    def start_simulation(self, network, sim_params):
        """Start a sumo simulation instance.

                This method performs the following operations:

                1. It collect the simulation step size and the emission path
                   information. If an emission path is specifies, it ensures that the
                   path exists.
                2. It also uses the configuration files created by the network class to
                   initialize a sumo instance.
                3. Finally, It initializes a traci connection to interface with sumo
                   from Python and returns the connection.
                """
        # Save the simulation step size (for later use).
        self.sim_step = sim_params.sim_step

        # Update the emission path term.
        self.emission_path = sim_params.emission_path
        if self.emission_path is not None:
            ensure_dir(self.emission_path)

        error = None
        for _ in range(RETRIES_ON_ERROR):
            try:
                # port number the sumo instance will be run on
                port = sim_params.port

                sumo_binary = "sumo-gui" if sim_params.render is True \
                    else "sumo"

                # command used to start sumo
                sumo_call = [
                    sumo_binary, "-c", network.cfg,
                    "--remote-port", str(sim_params.port),
                    "--num-clients", str(sim_params.num_clients),
                    "--step-length", str(sim_params.sim_step)
                ]

                # use a ballistic integration step (if request)
                if sim_params.use_ballistic:
                    sumo_call.append("--step-method.ballistic")

                # ignore step logs (if requested)
                if sim_params.no_step_log:
                    sumo_call.append("--no-step-log")

                # add the lateral resolution of the sublanes (if requested)
                if sim_params.lateral_resolution is not None:
                    sumo_call.append("--lateral-resolution")
                    sumo_call.append(str(sim_params.lateral_resolution))

                if sim_params.overtake_right:
                    sumo_call.append("--lanechange.overtake-right")
                    sumo_call.append("true")

                # specify a simulation seed (if requested)
                if sim_params.seed is not None:
                    sumo_call.append("--seed")
                    sumo_call.append(str(sim_params.seed))

                if not sim_params.print_warnings:
                    sumo_call.append("--no-warnings")
                    sumo_call.append("true")

                # set the time it takes for a gridlock teleport to occur
                sumo_call.append("--time-to-teleport")
                sumo_call.append(str(int(sim_params.teleport_time)))

                # check collisions at intersections
                sumo_call.append("--collision.check-junctions")
                sumo_call.append("true")

                logging.info(" Starting SUMO on port " + str(port))
                logging.debug(" Cfg file: " + str(network.cfg))
                if sim_params.num_clients > 1:
                    logging.info(" Num clients are" +
                                 str(sim_params.num_clients))
                logging.debug(" Emission file: " + str(self.emission_path))
                logging.debug(" Step length: " + str(sim_params.sim_step))

                print(sumo_call)

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

                traci_connection = traci.connect(port, numRetries=100)
                traci_connection.setOrder(0)
                traci_connection.simulationStep()

                return traci_connection
            except Exception as e:
                print("Error during start: {}".format(traceback.format_exc()))
                error = e
                self.teardown_sumo()
        raise error

    def teardown_sumo(self):
        """Kill the sumo subprocess instance."""
        try:
            os.killpg(self.sumo_proc.pid, signal.SIGTERM)
        except Exception as e:
            print("Error during teardown: {}".format(e))
