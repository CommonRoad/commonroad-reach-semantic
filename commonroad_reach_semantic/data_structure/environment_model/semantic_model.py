import logging
import warnings
from collections import defaultdict
from typing import List, Dict, Union

import commonroad_reach.utility.logger as util_logger
import numpy as np
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.traffic_sign import TrafficLightDirection, TrafficLightState

import commonroad_reach_semantic.utility.reach_operation as reach_operation
import commonroad_reach_semantic.utility.region as util_region
from commonroad_reach_semantic import pycrreachs
from commonroad_reach_semantic.data_structure.environment_model.lanelet_model import LaneletModel
from commonroad_reach_semantic.data_structure.environment_model.position_interval import PositionInterval
from commonroad_reach_semantic.data_structure.environment_model.region_model import RegionModel
from commonroad_reach_semantic.data_structure.environment_model.vehicle_model import VehicleModel
from commonroad_reach_semantic.data_structure.rule.proposition import Proposition as P
from commonroad_reach_semantic.data_structure.rule.proposition import PropositionGroup as PG
from commonroad_reach_semantic.data_structure.reach.semantic_reach_node import SemanticReachNode
from commonroad_reach_semantic.data_structure.environment_model.region import Region
from commonroad_reach_semantic.data_structure.environment_model.road_network import RoadNetwork
from commonroad_reach_semantic.data_structure.config.semantic_configuration import SemanticConfiguration

logger = logging.getLogger(__name__)


class SemanticModel:
    """
    Class to represent the semantic model of a given CommonRoad scenario.
    """

    def __init__(self, config: SemanticConfiguration):
        """
        Steps:
            1. create a smaller lanelet network and build a road network from it
            2. create vehicle objects from dynamic obstacles within the fov of the ego vehicle
            3. extract longitudinal/lateral position intervals from vehicles
            4. create lanelet regions
            5. extract traffic status propositions
        """
        logger.info("Creating SemanticModel...")

        self.config = config
        self.step_start = self.config.planning.step_start
        self.step_end = self.step_start + self.config.planning.steps_computation

        # proposition-related
        self.dict_step_to_traffic_status_propositions = dict()

        self.lanelet_model = LaneletModel(self.config)
        self.vehicle_model = VehicleModel(self.config, self.lanelet_model, self.step_start, self.step_end)
        self.region_model = RegionModel(self.config, self.lanelet_model, self.vehicle_model, self.step_end)
        self._determine_traffic_status_propositions()

        logger.info("SemanticModel created.")
        self.print_summary()

    def print_summary(self):
        string = "# ========= Model Summary ========= #\n"
        string += f"#\tLanes: {len(self.lanelet_model.road_network.list_lanes)}\n"
        string += f"#\tVehicles: {len(self.vehicle_model.list_vehicles)}\n"
        string += f"#\tRegions: {len(self.region_model.list_regions)}\n"
        string += "# ================================= #"

        for line in string.split("\n"):
            util_logger.print_and_log_info(logger, line)

    def _determine_traffic_status_propositions(self):
        """
        Determines propositions for the general traffic status.
        """
        dict_step_to_traffic_status_propositions = defaultdict(set)

        # extract propositions indicating a vehicle is within an intersection
        for id_lanelet in self.lanelet_model.set_ids_lanelets_in_intersections:
            lanelet = self.config.scenario.lanelet_network.find_lanelet_by_id(id_lanelet)

            for step in range(self.step_end + 1):
                set_ids_obstacles_dynamic = lanelet.dynamic_obstacle_by_time_step(step)

                if set_ids_obstacles_dynamic:
                    for id_obstacle in set_ids_obstacles_dynamic:
                        dict_step_to_traffic_status_propositions[step].add(P.in_intersection(id_obstacle))

        # extract propositions indicating a vehicle is in its outgoing lanelet
        for vehicle in self.vehicle_model.list_vehicles:
            for step in range(self.step_end + 1):
                if vehicle.set_ids_lanelets_successor_incoming.intersection(vehicle.lanelet_ids_at_step(step)):
                    dict_step_to_traffic_status_propositions[step].add(
                        eval(f"P.in_{vehicle.type_outgoing}_successor(vehicle.id_vehicle)"))

        self.dict_step_to_traffic_status_propositions = dict_step_to_traffic_status_propositions

    def determine_traffic_priorities(self, dict_traffic_sign_to_priorities: Dict):
        """
        Determines the traffic priorities for regions and vehicles.
        """
        if self.config.semantic_model.incoming_element_route:
            # vehicles
            for vehicle in self.vehicle_model.list_vehicles:
                vehicle.determine_priorities(dict_traffic_sign_to_priorities)

            # lanelet regions
            for region in self.region_model.list_regions:
                region.determine_priorities(dict_traffic_sign_to_priorities)
                region.examine_priorities_against_vehicles(self.vehicle_model.list_vehicles)

            logger.info("Traffic priorities determined.")

    def find_vehicle_by_id(self, id_vehicle: int):
        """
        Returns the vehicle object by its id.
        """
        for vehicle in self.vehicle_model.list_vehicles:
            if vehicle.id_vehicle == id_vehicle:
                return vehicle

    def label_traffic_propositions(self, step,
                                   list_propagated_sets: Union[
                                       List[SemanticReachNode], List[pycrreachs.SemanticReachNode]]) \
            -> Union[List[SemanticReachNode], List[pycrreachs.SemanticReachNode]]:
        """
        Labels propagated sets with propositions related to traffic status.
        """
        if not list_propagated_sets:
            return []

        list_propagated_sets = self.label_traffic_status_propositions(step, list_propagated_sets)
        list_propagated_sets = self.label_in_conflict_area_propositions(step, list_propagated_sets)
        list_propagated_sets = self.label_causes_braking_propositions(step, list_propagated_sets)

        return list_propagated_sets

    def label_traffic_status_propositions(self, step,
                                          list_propagated_sets: Union[
                                              List[SemanticReachNode], List[pycrreachs.SemanticReachNode]]) \
            -> Union[List[SemanticReachNode], List[pycrreachs.SemanticReachNode]]:
        """
        Labels propagated sets with traffic status propositions.
        """
        set_propositions = self.dict_step_to_traffic_status_propositions[step]
        # if isinstance(list_propagated_sets[0], ReachNode):
        for propagated_set in list_propagated_sets:
            propagated_set.proposition_holder.add_propositions(set_propositions, PG.TRAFFIC_STATUS)
        #
        # else:
        #     for propagated_set in list_propagated_sets:
        #         propagated_set.proposition_holder.add_propositions(set_propositions, "TRAFFIC_STATUS")

        return list_propagated_sets

    def label_in_conflict_area_propositions(self, step,
                                            list_propagated_sets: Union[
                                                List[SemanticReachNode], List[pycrreachs.SemanticReachNode]]) \
            -> Union[List[SemanticReachNode], List[pycrreachs.SemanticReachNode]]:
        """
        Labels propagated sets with propositions related to conflict status between them and the vehicles.

        Note: this relationship is asymmetric, refer to Sebastian's intersection traffic rule paper for definition.
        A lanelet is examined against a list of lanelets of the lane/route of the other object.
        """
        # examine if the propagated set is conflicting with the vehicles
        for propagated_set in list_propagated_sets:
            for vehicle in self.vehicle_model.list_vehicles:
                # if not vehicle.behind_node_at_step(step, base_set):
                #     continue

                # iterate through lanelet ids of the region
                for id_lanelet_propagated_set in propagated_set.set_ids_lanelets:
                    # iterate through lanelet ids of the lane of the vehicle
                    for id_lanelet_lane_vehicle in vehicle.lane.list_ids_lanelets:
                        # use cached results
                        if id_lanelet_lane_vehicle in \
                                self.lanelet_model.dict_id_lanelet_to_set_ids_lanelets_intersecting[id_lanelet_propagated_set]:
                            propagated_set.proposition_holder.add_propositions({P.in_conflict_with(vehicle.id_vehicle)},
                                                                               PG.TRAFFIC_STATUS)

        # examine if the vehicles are in conflict with the propagated set
        for propagated_set in list_propagated_sets:
            if isinstance(propagated_set, SemanticReachNode):
                p_lon_min = propagated_set.p_lon_min
            else:
                p_lon_min = propagated_set.p_lon_min()

            for vehicle in self.vehicle_model.list_vehicles:
                try:
                    p_lon_min_propagated_set = p_lon_min - self.config.vehicle.ego.radius_inflation
                    p_lon_ref_max_vehicle = vehicle.dict_step_to_state_lon_ref[step].s + vehicle.shape.length / 2

                except (AttributeError, KeyError):
                    continue

                # propagated set is in front of the vehicle along the reference path
                if p_lon_min_propagated_set > p_lon_ref_max_vehicle:
                    continue

                # iterate through lanelet ids of the route
                for id_lanelet_route in self.config.planning.route.list_ids_lanelets:
                    # iterate through lanelet ids of the vehicle
                    for id_lanelet_vehicle in vehicle.lanelet_ids_at_step(step):
                        if id_lanelet_vehicle in \
                                self.lanelet_model.dict_id_lanelet_to_set_ids_lanelets_intersecting[id_lanelet_route]:
                            propagated_set.proposition_holder.add_propositions({P.in_conflict_by(vehicle.id_vehicle)},
                                                                               PG.VEHICLE)

        return list_propagated_sets

    def label_causes_braking_propositions(self, step: int,
                                          list_propagated_sets: Union[
                                              List[SemanticReachNode], List[pycrreachs.SemanticReachNode]]) \
            -> Union[List[SemanticReachNode], List[pycrreachs.SemanticReachNode]]:
        """
        Labels propagated sets with propositions related to causes braking to other vehicles.
        """
        # only compute if there is an intersection
        if not self.config.semantic_model.incoming_element_route:
            return list_propagated_sets

        for propagated_set in list_propagated_sets:
            # todo: this should only be computed for vehicles entering an intersection from other directions
            for vehicle in self.vehicle_model.list_vehicles:
                if vehicle.braking_caused_by_node_at_step(step, propagated_set):
                    propagated_set.proposition_holder.add_propositions({P.causes_braking_for(vehicle.id_vehicle)},
                                                                       PG.TRAFFIC_STATUS)

        return list_propagated_sets

    def split_wrt_regions(self, step: int, reachable_set: SemanticReachNode) -> List[SemanticReachNode]:
        """
        Splits a reachable set w.r.t lanelet regions.

        Steps:
            1. Intersect reachable set in the position domain with lanelet regions
            2. Over-approximate and restore to axis-aligned rectangles
        """
        list_sets_split = []
        # iterate through regions intersecting with the reachable set
        for region in self.region_model.list_regions:
            # first compute intersection with bounding box
            # --> exact intersection is more expensive, so we only want to compute it if necessary?
            # there is no possibility of intersection
            if not region.intersects(reachable_set.position_rectangle.bounds, coordinate_system="CVLN"):
                continue

            # there is a possibility of intersection
            # TODO: Find out, why there was a try-except for Exception here
            polygon_intersection = region.polygon_cvln.intersection(reachable_set.position_rectangle)

            # empty intersection
            if polygon_intersection.is_empty:
                continue

            # over-approximate by restoring the intersected polygon to axis-aligned rectangle
            bounds_polygon_intersection = polygon_intersection.bounds

            # clone the propagated set and split in the position domain, update the propositions
            # TODO: Find out, why there was a try-except for AttributeError here
            reachable_set_new = reachable_set.clone()
            reachable_set_new.intersect_in_position_domain(*bounds_polygon_intersection)
            reachable_set_new = self.update_propositions_with_region(reachable_set_new, region, step)

            list_sets_split.append(reachable_set_new)

        return list_sets_split

    def update_propositions_with_region(self, propagated_set: Union[SemanticReachNode, pycrreachs.SemanticReachNode],
                                        region: Union[Region, pycrreachs.Region], step: int):
        """
        Updates the propositions of the propagated set with the proposition of the lanelet region.

        Lanelet transition proposition are considered to be temporary since they are only used for TPL compliance
        checking and should be omitted during merging of the propagated sets.
        """
        # add propositions of the region to persistent propositions of the propagated set
        if isinstance(region, Region):
            dict_relevant = region.dict_group_to_propositions_at_step(step)

        else:
            dict_relevant = region.map_group_to_propositions_at_step(step)

        for group in dict_relevant:
            set_propositions = dict_relevant[group]
            propagated_set.proposition_holder.add_propositions(set_propositions, group)

        # add lanelet ids of the region to propagated set
        if isinstance(propagated_set, SemanticReachNode):
            propagated_set.set_ids_lanelets.update(region.set_ids_lanelets)

        else:
            propagated_set.add_lanelet_ids(region.set_ids_lanelets)

        # add lanelet transition as temporary propositions
        set_propositions = self.obtain_lanelet_transition_propositions(propagated_set)
        propagated_set.proposition_holder.add_propositions(set_propositions, PG.TEMPORARY)

        return propagated_set

    def split_wrt_position_intervals(self, step: int,
                                     reachable_set: SemanticReachNode) -> List[SemanticReachNode]:
        """
        Splits the reachable set w.r.t position intervals.
        """

        list_intervals_lon: List[PositionInterval] = self.vehicle_model.dict_step_to_position_intervals[step]["lon"]
        list_intervals_lat: List[PositionInterval] = self.vehicle_model.dict_step_to_position_intervals[step]["lat"]

        list_reachable_sets_split_lon = []
        for interval_lon in list_intervals_lon:
            # propagated set intersects with the longitudinal interval
            if interval_lon.intersects(reachable_set.p_lon_min, reachable_set.p_lon_max):
                propagated_set_split = reach_operation.split_reach_node_to_interval(reachable_set, interval_lon, "lon")
                if propagated_set_split:
                    list_reachable_sets_split_lon.append(propagated_set_split)

            # early termination, since the rest of intervals will definitely not intersect with the base set
            elif interval_lon.p_min > reachable_set.polygon_lon.p_max:
                break

        list_reachable_sets_split = []
        for propagated in list_reachable_sets_split_lon:
            for interval_lat in list_intervals_lat:
                # propagated set intersects with the lateral interval
                if interval_lat.intersects(propagated.p_lat_min, propagated.p_lat_max):
                    propagated_set_split = reach_operation.split_reach_node_to_interval(propagated, interval_lat, "lat")
                    if propagated_set_split:
                        list_reachable_sets_split.append(propagated_set_split)

                # early termination, since the rest of intervals will definitely not intersect with the base set
                elif interval_lat.p_min > propagated.polygon_lat.p_max:
                    break

        return list_reachable_sets_split

    @staticmethod
    def obtain_lanelet_transition_propositions(propagated_set: Union[SemanticReachNode, pycrreachs.SemanticReachNode]):
        """
        Returns the set of lanelet transition propositions.
        """
        set_propositions = set()
        # retrieve lanelet propositions from the source
        if isinstance(propagated_set, SemanticReachNode):
            set_propositions_position_source = \
                propagated_set.source_propagation.proposition_holder.propositions_in_group(group=PG.POSITION)

        else:
            set_propositions_position_source = \
                propagated_set.vec_nodes_source[0].proposition_holder.propositions_in_group(PG.POSITION)

        set_ids_lanelets_source = {int(proposition.split("_")[1]) for proposition in set_propositions_position_source
                                   if P.in_lanelet() in proposition}

        # generate lanelet transition propositions
        for id_lanelet_source in set_ids_lanelets_source:
            for id_lanelet_base_set in propagated_set.set_ids_lanelets:
                if id_lanelet_source != id_lanelet_base_set:
                    set_propositions.add(P.lanelet_transition(id_lanelet_source, id_lanelet_base_set))

        return set_propositions

    @staticmethod
    def discard_colliding_nodes(list_propagated_set: List[SemanticReachNode]) -> List[SemanticReachNode]:
        """
        Returns a list of propagated sets that do not collide with vehicles.
        """
        list_nodes_keep = []

        for propagated_set in list_propagated_set:
            colliding = False
            set_propositions = propagated_set.set_propositions()
            for proposition in set_propositions:
                # check if it is aligned with and besides a vehicle
                if P.aligned_with() in proposition:
                    id_vehicle = int(proposition.split("_")[1][1:])

                    if P.beside(id_vehicle) in set_propositions:
                        colliding = True
                        break

            if not colliding:
                list_nodes_keep.append(propagated_set)

        return list_nodes_keep

    @staticmethod
    def call_python_dummy(step, node):
        """
        Dummy function to be called from C++.
        """
        return node
