from typing import Union, Optional, List

from commonroad.planning.goal import GoalRegion
from commonroad.planning.planning_problem import PlanningProblemSet, PlanningProblem
from commonroad.scenario.intersection import IntersectionIncomingElement
from commonroad.scenario.lanelet import LaneletType
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.state import State
from commonroad_dc.pycrccosy import CurvilinearCoordinateSystem
from commonroad_reach.data_structure.configuration import Configuration, ConfigurationBase, ReachableSetConfiguration
from omegaconf import ListConfig, DictConfig

import commonroad_reach_semantic_addon.utility.vehicle as util_vehicle


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


class SemanticReachableSetConfiguration(ReachableSetConfiguration):
    def __init__(self, config: Union[ListConfig, DictConfig]):
        super().__init__(config)
        self.length_edge_node_min = config.reachable_set.length_edge_node_min
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
                self.direction_outgoing = "left"

            elif id_lanelet_successor in self.incoming_element_route.successors_straight:
                self.direction_outgoing = "straight"

            elif id_lanelet_successor in self.incoming_element_route.successors_right:
                self.direction_outgoing = "right"

            # oncoming lanelet ids
            self.set_ids_lanelets_oncoming = \
                util_vehicle.extract_oncomings_from_incoming(self.incoming_element_route,
                                                             planning_config.lanelet_network)


class TrafficRuleConfiguration(ConfigurationBase):
    def __init__(self, config: Union[ListConfig, DictConfig]):
        config_relevant = config.traffic_rules

        self.distance_braking = config_relevant.distance_braking
        self.acceleration_braking_hard = config_relevant.acceleration_braking_hard
        self.activated_rules = config_relevant.activated_rules
        self.mode_spot = config_relevant.mode_spot
