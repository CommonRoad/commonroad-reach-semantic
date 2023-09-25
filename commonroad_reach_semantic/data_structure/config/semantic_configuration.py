import logging
from typing import Union, Optional, List

import commonroad_reach.utility.logger as util_logger
from commonroad.planning.goal import GoalRegion
from commonroad.planning.planning_problem import PlanningProblemSet, PlanningProblem
from commonroad.scenario.intersection import IntersectionIncomingElement
from commonroad.scenario.lanelet import LaneletType
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.state import State
from commonroad_dc.pycrccosy import CurvilinearCoordinateSystem
from commonroad_reach import pycrreach
from commonroad_reach.data_structure.configuration import Configuration, ConfigurationBase, ReachableSetConfiguration
from omegaconf import ListConfig, DictConfig

import commonroad_reach_semantic.utility.vehicle as util_vehicle
from commonroad_reach_semantic import pycrreachs
from commonroad_reach_semantic.data_structure.config.outgoing_direction import OutgoingDirection

logger = logging.getLogger(__name__)


class SemanticConfiguration(Configuration):

    def __init__(self, config_omega: Union[ListConfig, DictConfig]):
        super().__init__(config_omega)
        self.reachable_set: SemanticReachableSetConfiguration = SemanticReachableSetConfiguration(config_omega)
        self.sonia: SONIAConfiguration = SONIAConfiguration(config_omega)
        self.semantic_model: SemanticModelConfiguration = SemanticModelConfiguration(config_omega)
        self.traffic_rule: TrafficRuleConfiguration = TrafficRuleConfiguration(config_omega)

    def update(self, scenario: Scenario = None, planning_problem_set: PlanningProblemSet = None,
               planning_problem: PlanningProblem = None, idx_planning_problem: int = 0,
               state_initial: State = None, goal_region: GoalRegion = None,
               CLCS: CurvilinearCoordinateSystem = None, list_ids_lanelets: List[int] = None):
        super().update(scenario, planning_problem_set, planning_problem, idx_planning_problem, state_initial,
                       goal_region, CLCS, list_ids_lanelets)
        self.semantic_model.update_configuration(self)

    def print_configuration_summary(self):
        """Prints a summary of the configuration."""
        # TODO: could be nicer if we refactored the configuration system to be more modular
        # currently this is copied from commonroad_reach/data_structure/configuration.py and adapted
        dict_clcs_to_string = {"CART": "cartesian", "CVLN": "curvilinear"}
        dict_mode_computation_to_string = {1: "polytopic, python backend", 2: "polytopic, c++ backend",
                                           3: "graph-based (online)", 4: "graph-based (offline)",
                                           5: "polytopic, semantic labeling, python backend",
                                           6: "polytopic, semantic labeling, c++ backend",
                                           7: "polytopic, on-the-fly model checking, python backend",
                                           8: "polytopic, on-the-fly model checking, c++ backend"}
        dict_mode_repartition_to_string = {1: "repartition, collision check", 2: "collision check, repartition",
                                           3: "repartition, collision check, then repartition"}
        dict_mode_inflation_to_string = {1: "inscribed circle", 2: "circumscribed circle",
                                         3: "three circle approximation"}

        CLCS = dict_clcs_to_string[self.planning.coordinate_system]
        mode_computation = dict_mode_computation_to_string[self.reachable_set.mode_computation]
        mode_repartition = dict_mode_repartition_to_string[self.reachable_set.mode_repartition]
        mode_inflation = dict_mode_inflation_to_string[self.reachable_set.mode_inflation]

        string = "\n# ===== CommonRoad-Reach-Semantic Configuration Summary ===== #\n"
        string += f"# {self.scenario.scenario_id}\n"
        string += "# Planning:\n"
        string += f"# \tdt: {self.planning.dt}\n"
        string += f"# \tsteps: {self.planning.steps_computation}\n"
        string += f"# \tcoordinate system: {CLCS}\n"

        config_ego = self.vehicle.ego
        string += "# Vehicle (Ego):\n"
        string += f"# \tvehicle type id: {self.vehicle.ego.id_type_vehicle}\n"
        string += f"# \tv: lon_min = {config_ego.v_lon_min}, lon_max = {config_ego.v_lon_max}, " \
                  f"lat_min = {config_ego.v_lat_min}, lat_max = {config_ego.v_lat_max}, max = {config_ego.v_max}\n"
        string += f"# \ta: lon_min = {config_ego.a_lon_min}, lon_max = {config_ego.a_lon_max}, " \
                  f"lat_min = {config_ego.a_lat_min}, lat_max = {config_ego.a_lat_max}, max = {config_ego.a_max}\n"

        string += "# Reachable set:\n"
        string += f"# \tcomputation mode: {mode_computation}\n"
        string += f"# \trepartition mode: {mode_repartition}\n"
        string += f"# \tinflation mode: {mode_inflation}\n"
        string += f"# \tobstacle rasterization: {self.reachable_set.rasterize_obstacles}\n"
        string += f"# \tgrid size: {self.reachable_set.size_grid}\n"
        string += f"# \tsplit radius: {self.reachable_set.radius_terminal_split}\n"
        string += f"# \tprune: {self.reachable_set.prune_nodes_not_reaching_final_step}\n"
        string += f"# \tnum threads: {self.reachable_set.num_threads}\n"
        string += "# ================================= #"

        for line in string.split("\n"):
            util_logger.print_and_log_info(logger, line)

    def convert_to_cpp_configuration(self) -> pycrreachs.SemanticConfiguration:
        """
        Converts to a configuration that is readable by the C++ binding code.
        """
        # TODO: could be nicer if we had a function like write_cpp_config(self, cpp_config) -> None instead
        # in this case we could reuse the superclass method
        config = pycrreachs.SemanticConfiguration()

        config.general.name_scenario = self.name_scenario
        config.general.path_scenarios = self.general.path_scenarios

        config.vehicle.ego.id_type_vehicle = self.vehicle.ego.id_type_vehicle
        config.vehicle.ego.length = self.vehicle.ego.length
        config.vehicle.ego.width = self.vehicle.ego.width
        config.vehicle.ego.v_lon_min = self.vehicle.ego.v_lon_min
        config.vehicle.ego.v_lon_max = self.vehicle.ego.v_lon_max
        config.vehicle.ego.v_lat_min = self.vehicle.ego.v_lat_min
        config.vehicle.ego.v_lat_max = self.vehicle.ego.v_lat_max
        config.vehicle.ego.a_lon_min = self.vehicle.ego.a_lon_min
        config.vehicle.ego.a_lon_max = self.vehicle.ego.a_lon_max
        config.vehicle.ego.a_lat_min = self.vehicle.ego.a_lat_min
        config.vehicle.ego.a_lat_max = self.vehicle.ego.a_lat_max
        config.vehicle.ego.a_max = self.vehicle.ego.a_max
        config.vehicle.ego.radius_disc = self.vehicle.ego.radius_disc
        config.vehicle.ego.circle_distance = self.vehicle.ego.circle_distance
        config.vehicle.ego.wheelbase = self.vehicle.ego.wheelbase

        config.vehicle.other.id_type_vehicle = self.vehicle.other.id_type_vehicle
        config.vehicle.other.length = self.vehicle.other.length
        config.vehicle.other.width = self.vehicle.other.width
        config.vehicle.other.v_lon_min = self.vehicle.other.v_lon_min
        config.vehicle.other.v_lon_max = self.vehicle.other.v_lon_max
        config.vehicle.other.v_lat_min = self.vehicle.other.v_lat_min
        config.vehicle.other.v_lat_max = self.vehicle.other.v_lat_max
        config.vehicle.other.a_lon_min = self.vehicle.other.a_lon_min
        config.vehicle.other.a_lon_max = self.vehicle.other.a_lon_max
        config.vehicle.other.a_lat_min = self.vehicle.other.a_lat_min
        config.vehicle.other.a_lat_max = self.vehicle.other.a_lat_max
        config.vehicle.other.a_max = self.vehicle.other.a_max
        config.vehicle.other.radius_disc = self.vehicle.other.radius_disc
        config.vehicle.other.circle_distance = self.vehicle.other.circle_distance
        config.vehicle.other.wheelbase = self.vehicle.other.wheelbase

        config.planning.dt = self.planning.dt
        config.planning.step_start = self.planning.step_start
        config.planning.steps_computation = self.planning.steps_computation
        config.planning.p_lon_initial = self.planning.p_lon_initial
        config.planning.p_lat_initial = self.planning.p_lat_initial
        config.planning.uncertainty_p_lon = self.planning.uncertainty_p_lon
        config.planning.uncertainty_p_lat = self.planning.uncertainty_p_lat
        config.planning.v_lon_initial = self.planning.v_lon_initial
        config.planning.v_lat_initial = self.planning.v_lat_initial
        config.planning.uncertainty_v_lon = self.planning.uncertainty_v_lon
        config.planning.uncertainty_v_lat = self.planning.uncertainty_v_lat

        if self.planning.coordinate_system == "CART":
            config.planning.coordinate_system = pycrreach.CoordinateSystem.CARTESIAN

        else:
            config.planning.coordinate_system = pycrreach.CoordinateSystem.CURVILINEAR
            config.planning.CLCS = self.planning.CLCS

        if self.planning.reference_point == "REAR":
            config.planning.reference_point = pycrreach.ReferencePoint.REAR

        else:
            config.planning.reference_point = pycrreach.ReferencePoint.CENTER

        config.reachable_set.mode_repartition = self.reachable_set.mode_repartition
        config.reachable_set.mode_inflation = self.reachable_set.mode_inflation
        config.reachable_set.size_grid = self.reachable_set.size_grid
        config.reachable_set.size_grid_2nd = self.reachable_set.size_grid_2nd
        config.reachable_set.radius_terminal_split = self.reachable_set.radius_terminal_split
        config.reachable_set.num_threads = self.reachable_set.num_threads
        config.reachable_set.prune_nodes = self.reachable_set.prune_nodes_not_reaching_final_step
        config.reachable_set.rasterize_obstacles = self.reachable_set.rasterize_obstacles

        # convert lut dict to Cpp configuration via PyBind function
        if self.reachable_set.mode_inflation == 3:
            config.reachable_set.lut_lon_enlargement = \
                pycrreach.LUTLongitudinalEnlargement(self.reachable_set.lut_longitudinal_enlargement)

        config.traffic_rule.distance_braking = self.traffic_rule.distance_braking
        config.traffic_rule.acceleration_braking_hard = self.traffic_rule.acceleration_braking_hard
        config.traffic_rule.backward_driving_v_err = self.traffic_rule.backward_driving_v_err
        config.traffic_rule.activated_rules = self.traffic_rule.activated_rules
        config.traffic_rule.mode_spot = self.traffic_rule.mode_spot
        config.traffic_rule.mode_automata = self.traffic_rule.mode_automata

        config.semantic_model.is_intersection = self.semantic_model.incoming_element_route is not None
        config.semantic_model.ego_radius_inflation = self.vehicle.ego.radius_inflation
        config.semantic_model.vec_route_lanelet_ids = self.planning.route.list_ids_lanelets

        return config


class SemanticReachableSetConfiguration(ReachableSetConfiguration):
    def __init__(self, config: Union[ListConfig, DictConfig]):
        super().__init__(config)
        self.num_corridors_max = config.reachable_set.num_corridors_max


class SONIAConfiguration:
    def __init__(self, config: Union[ListConfig, DictConfig]):
        config_relevant = config.sonia

        self.time_steps_computation_extra = config_relevant.time_steps_computation_extra

        self.compute_assumption_m1 = config_relevant.compute_assumption_m1
        self.compute_assumption_m2 = config_relevant.compute_assumption_m2
        self.compute_assumption_m3 = config_relevant.compute_assumption_m3

        self.consider_occlusion = config_relevant.consider_occlusion
        self.print_operation_status = config_relevant.print_operation_status
        self.print_predicted_velocity_intervals = config_relevant.print_predicted_velocity_intervals
        self.update_obstacle_parameters = config_relevant.update_obstacle_parameters
        self.num_threads = config_relevant.num_threads


class SemanticModelConfiguration(ConfigurationBase):
    def __init__(self, config: Union[ListConfig, DictConfig]):
        config_relevant = config.semantic_model

        self.p_lateral_max = config_relevant.p_lateral_max
        self.length_aabb_max = config_relevant.length_aabb_max
        self.discard_region_small = config_relevant.discard_region_small
        self.area_polygon_desired_min = config_relevant.area_polygon_desired_min
        self.buffer_polygon = config_relevant.buffer_polygon
        self.consider_ego_shape = config_relevant.consider_ego_shape
        self.consider_route_predecessor = config_relevant.consider_route_predecessor
        self.consider_route_successor = config_relevant.consider_route_successor
        self.distance_vertices_polygon_max = config_relevant.distance_vertices_polygon_max

        self.use_sonia = config_relevant.use_sonia

        # intersection-related elements
        self.incoming_element_route: Optional[IntersectionIncomingElement] = None
        self.direction_outgoing = None
        self.set_ids_lanelets_oncoming = set()

    def update_configuration(self, config: SemanticConfiguration):
        planning_config = config.planning

        # ==== intersection-related attributes for routes passing through an intersection
        id_lanelet_incoming = id_lanelet_successor = None
        for id_lanelet_pre, id_lanelet_suc in zip(planning_config.route.list_ids_lanelets[:-1],
                                                  planning_config.route.list_ids_lanelets[1:]):
            lanelet_pre = config.scenario.lanelet_network.find_lanelet_by_id(id_lanelet_pre)
            lanelet_suc = config.scenario.lanelet_network.find_lanelet_by_id(id_lanelet_suc)

            # two consecutive lanelets of the type intersection
            if LaneletType.INTERSECTION in lanelet_pre.lanelet_type and \
                    LaneletType.INTERSECTION in lanelet_suc.lanelet_type:
                id_lanelet_incoming = id_lanelet_pre
                id_lanelet_successor = id_lanelet_suc
                break

        # if there are two consecutive intersection lanelets along the route, locate the correct incoming element
        if id_lanelet_incoming and id_lanelet_successor:
            for intersection in planning_config.lanelet_network.intersections:
                for incoming_element in intersection.incomings:
                    if id_lanelet_incoming in incoming_element.incoming_lanelets:
                        self.incoming_element_route = incoming_element
                        break

        # if there is an incoming element, identify the outgoing direction
        if self.incoming_element_route:
            if id_lanelet_successor in self.incoming_element_route.successors_left:
                self.direction_outgoing = OutgoingDirection.LEFT

            elif id_lanelet_successor in self.incoming_element_route.successors_straight:
                self.direction_outgoing = OutgoingDirection.STRAIGHT

            elif id_lanelet_successor in self.incoming_element_route.successors_right:
                self.direction_outgoing = OutgoingDirection.RIGHT

            # oncoming lanelet ids
            self.set_ids_lanelets_oncoming = \
                util_vehicle.extract_oncomings_from_incoming(self.incoming_element_route,
                                                             planning_config.lanelet_network)


class TrafficRuleConfiguration(ConfigurationBase):
    def __init__(self, config: Union[ListConfig, DictConfig]):
        config_relevant = config.traffic_rules

        self.distance_braking = config_relevant.distance_braking
        self.acceleration_braking_hard = config_relevant.acceleration_braking_hard
        self.backward_driving_v_err = config_relevant.backward_driving_v_err
        self.activated_rules = config_relevant.activated_rules
        self.mode_spot = config_relevant.mode_spot
        self.mode_automata = config_relevant.mode_automata
