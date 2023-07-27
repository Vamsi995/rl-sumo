import traci.constants as tc
from rlsumo.vehicle.controllers.controller import IDMControl, RLControl, FollowerStopper, PISaturation


class VehicleParams:

    def __init__(self, env_vehicles=21, rl_vehicles=1, max_velocity=30, initial_velocity=0, length=5, track_length=260,
                 rl_action_type="discrete", agent_type="idm"):
        self.env_vehicles = env_vehicles
        self.rl_vehicles = rl_vehicles
        self.max_velocity = max_velocity
        self.initial_velocity = initial_velocity
        self.length = length
        self.track_length = track_length
        self.vehicle_subs = [tc.VAR_FUELCONSUMPTION, tc.VAR_SPEED, tc.VAR_ROAD_ID, tc.VAR_ACCELERATION,
                             tc.VAR_LANEPOSITION, tc.VAR_DISTANCE]
        self.edge_len = track_length / 4.
        self.rts = {
            "top": ["top", "left", "bottom", "right"],
            "left": ["left", "bottom", "right", "top"],
            "bottom": ["bottom", "right", "top", "left"],
            "right": ["right", "top", "left", "bottom"]
        }
        self.lane_positions = {
            "bottom": 0,
            "right": 1 * self.edge_len,
            "top": 2 * self.edge_len,
            "left": 3 * self.edge_len
        }
        self.controller = {
            "idm": IDMControl(noise=0.1),
            "fs": FollowerStopper(),
            "pi": PISaturation(),
            "rl": RLControl()
        }
        self.agent_type = agent_type
        self.human_veh = "human"
        self.automated_veh = "automated"
        self.rl_action_type = rl_action_type


class SimulationParams:

    def __init__(self,
                 port=8813,
                 sim_step=0.1,
                 emission_path=None,
                 lateral_resolution=None,
                 no_step_log=True,
                 render=False,
                 save_render=False,
                 sight_radius=25,
                 show_radius=False,
                 pxpm=2,
                 force_color_update=False,
                 overtake_right=False,
                 seed=None,
                 restart_instance=False,
                 print_warnings=True,
                 teleport_time=-1,
                 num_clients=1,
                 color_by_speed=False,
                 use_ballistic=False):
        self.port = port
        self.render = render
        self.sim_step = sim_step
        self.emission_path = emission_path
        self.lateral_resolution = lateral_resolution
        self.no_step_log = no_step_log
        self.seed = seed
        self.overtake_right = overtake_right
        self.print_warnings = print_warnings
        self.teleport_time = teleport_time
        self.num_clients = num_clients
        self.color_by_speed = color_by_speed
        self.use_ballistic = use_ballistic
        self.sim_config_path = '/home/vamsi/Documents/GitHub/rl-sumo/rlsumo/simulator/sumo_config/cfg/ring_road_20230617-1140031686982203.6779535.sumo.cfg'


class RLParams:

    def __init__(self, max_episodes=3000, warmup_steps=3000, discount_factor=0.99, env_horizon=3000):
        self.max_episodes = 3000
        self.warmup_steps = warmup_steps
        self.discount_factor = discount_factor
        self.env_horizon = env_horizon


class Params:

    def __init__(self, veh, rl, sim):
        self.vehicle_params = veh
        self.rl_params = rl
        self.simulation_params = sim
