import itertools
from collections import defaultdict
from typing import Union, List, Dict, FrozenSet

from commonroad_reach.data_structure.reach.reach_node import ReachNode
from commonroad_reach.pycrreach import ReachPolygon

from commonroad_reach_semantic import pycrreachs
from commonroad_reach_semantic.data_structure.environment_model.position_interval import PositionInterval
from commonroad_reach_semantic.data_structure.environment_model.region import Region
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.rule.proposition import Proposition as Prop
from commonroad_reach_semantic.data_structure.rule.proposition import PropositionGroup as PropGroup
from commonroad_reach_semantic.data_structure.rule.proposition_holder import PropositionHolder
from commonroad_reach_semantic.utility import reach_operation


class ReachableSetLabeler:
    """Splits reachable sets and labels the parts according to the semantic model."""
    semantic_model: SemanticModel
    reachable_set_to_propositions: Dict[ReachNode, PropositionHolder]
    reachable_set_to_lanelet_ids: Dict[ReachNode, FrozenSet[int]]

    def __init__(self, semantic_model: SemanticModel):
        self.semantic_model = semantic_model
        self.reachable_set_to_propositions = defaultdict(PropositionHolder)
        self.reachable_set_to_lanelet_ids = dict()

    def label_initial_state(self, reachable_sets: List[ReachNode], step_start: int) -> None:
        """
        Assigns proposition labels to initial reachable sets and drivable areas.
        """
        for reachable_set in reachable_sets:
            drivable_area = reachable_set.position_rectangle
            propositions, set_ids_lanelets = self._obtain_propositions_for_rectangle(drivable_area, step_start)
            self.reachable_set_to_propositions[reachable_set].merge(propositions)
            self.reachable_set_to_lanelet_ids[reachable_set] = set_ids_lanelets
        self.label_traffic_propositions(step_start, reachable_sets)

    def _obtain_propositions_for_rectangle(self, rectangle: ReachPolygon, step: int) -> tuple[
        PropositionHolder, FrozenSet[int]]:
        """
        Returns the propositions of the given rectangle.

        Intersects the rectangle with regions and position intervals. Since this method does not split the rectangle,
        it adds the propositions of the first intersecting region and position interval.
        """
        proposition_holder = PropositionHolder()
        set_ids_lanelets = frozenset()
        # retrieve propositions from the intersecting lanelet region
        for region in self.semantic_model.region_model.list_regions:
            if region.polygon_cvln.intersects(rectangle):
                for group, set_propositions in region.dict_group_to_propositions_at_step(step).items():
                    proposition_holder.add_propositions(set_propositions, group)
                set_ids_lanelets = frozenset(region.set_ids_lanelets)
                break

        # retrieve vehicle-related propositions from position intervals
        list_intervals_lon = self.semantic_model.vehicle_model.dict_step_to_position_intervals[step]["lon"]
        list_intervals_lat = self.semantic_model.vehicle_model.dict_step_to_position_intervals[step]["lat"]

        for interval_lon in list_intervals_lon:
            if interval_lon.intersects(rectangle.p_lon_min, rectangle.p_lon_max):
                proposition_holder.add_propositions(interval_lon.set_propositions, PropGroup.POSITION)
                break

        for interval_lat in list_intervals_lat:
            if interval_lat.intersects(rectangle.p_lat_min, rectangle.p_lat_max):
                proposition_holder.add_propositions(interval_lat.set_propositions, PropGroup.POSITION)
                break

        return proposition_holder, set_ids_lanelets

    def label_traffic_propositions(self, step,
                                   list_propagated_sets: Union[
                                       List[ReachNode], List[pycrreachs.SemanticReachNode]]) \
            -> Union[List[ReachNode], List[pycrreachs.SemanticReachNode]]:
        """
        Labels propagated sets with propositions related to traffic status.
        """
        if not list_propagated_sets:
            return []

        list_propagated_sets = self._label_traffic_status_propositions(step, list_propagated_sets)
        list_propagated_sets = self._label_in_conflict_area_propositions(step, list_propagated_sets)
        list_propagated_sets = self._label_causes_braking_propositions(step, list_propagated_sets)

        return list_propagated_sets

    def _label_traffic_status_propositions(self, step,
                                           list_propagated_sets: Union[
                                               List[ReachNode], List[pycrreachs.SemanticReachNode]]) \
            -> Union[List[ReachNode], List[pycrreachs.SemanticReachNode]]:
        """
        Labels propagated sets with traffic status propositions.
        """
        set_propositions = self.semantic_model.traffic_status_model.dict_step_to_traffic_status_propositions[step]
        for propagated_set in list_propagated_sets:
            self.reachable_set_to_propositions[propagated_set].add_propositions(set_propositions,
                                                                                PropGroup.TRAFFIC_STATUS)

        return list_propagated_sets

    def _label_in_conflict_area_propositions(self, step,
                                             list_propagated_sets: Union[
                                                 List[ReachNode], List[pycrreachs.SemanticReachNode]]) \
            -> Union[List[ReachNode], List[pycrreachs.SemanticReachNode]]:
        """
        Labels propagated sets with propositions related to conflict status between them and the vehicles.

        Note: this relationship is asymmetric, refer to Sebastian's intersection traffic rule paper for definition.
        A lanelet is examined against a list of lanelets of the lane/route of the other object.
        """
        # examine if the propagated set is conflicting with the vehicles
        for propagated_set, vehicle in itertools.product(list_propagated_sets,
                                                         self.semantic_model.vehicle_model.list_vehicles):
            # iterate through lanelet ids of the region and lanelet ids of the lane of the vehicle
            for id_lanelet_propagated_set, id_lanelet_lane_vehicle in itertools.product(
                    self.reachable_set_to_lanelet_ids[propagated_set],
                    vehicle.lane.list_ids_lanelets):
                # use cached results
                if id_lanelet_lane_vehicle in \
                        self.semantic_model.lanelet_model.dict_id_lanelet_to_set_ids_lanelets_intersecting[
                            id_lanelet_propagated_set]:
                    self.reachable_set_to_propositions[propagated_set].add_proposition(
                        Prop.in_conflict_with(vehicle.id_vehicle),
                        PropGroup.TRAFFIC_STATUS)

        # examine if the vehicles are in conflict with the propagated set
        for propagated_set in list_propagated_sets:
            if isinstance(propagated_set, ReachNode):
                p_lon_min = propagated_set.p_lon_min
            else:
                p_lon_min = propagated_set.p_lon_min()

            for vehicle in self.semantic_model.vehicle_model.list_vehicles:
                try:
                    p_lon_min_propagated_set = p_lon_min - self.semantic_model.config.vehicle.ego.radius_inflation
                    p_lon_ref_max_vehicle = vehicle.dict_step_to_state_lon_ref[step].s + vehicle.shape.length / 2

                except (AttributeError, KeyError):
                    continue

                # propagated set is in front of the vehicle along the reference path
                if p_lon_min_propagated_set > p_lon_ref_max_vehicle:
                    continue

                # iterate through lanelet ids of the route and lanelet ids of the vehicle
                for id_lanelet_route, id_lanelet_vehicle in itertools.product(
                        self.semantic_model.config.planning.route.list_ids_lanelets, vehicle.lanelet_ids_at_step(step)):
                    if id_lanelet_vehicle in \
                            self.semantic_model.lanelet_model.dict_id_lanelet_to_set_ids_lanelets_intersecting[
                                id_lanelet_route]:
                        self.reachable_set_to_propositions[propagated_set].add_proposition(
                            Prop.in_conflict_by(vehicle.id_vehicle),
                            PropGroup.VEHICLE)

        return list_propagated_sets

    def _label_causes_braking_propositions(self, step: int,
                                           list_propagated_sets: Union[
                                               List[ReachNode], List[pycrreachs.SemanticReachNode]]) \
            -> Union[List[ReachNode], List[pycrreachs.SemanticReachNode]]:
        """
        Labels propagated sets with propositions related to causes braking to other vehicles.
        """
        # only compute if there is an intersection
        if not self.semantic_model.config.semantic_model.incoming_element_route:
            return list_propagated_sets

        # todo: this should only be computed for vehicles entering an intersection from other directions
        for propagated_set, vehicle in itertools.product(list_propagated_sets,
                                                         self.semantic_model.vehicle_model.list_vehicles):
            if vehicle.braking_caused_by_node_at_step(step, propagated_set):
                self.reachable_set_to_propositions[propagated_set].add_proposition(
                    Prop.causes_braking_for(vehicle.id_vehicle),
                    PropGroup.TRAFFIC_STATUS)

        return list_propagated_sets

    def split_wrt_regions(self, step: int, reachable_set: ReachNode) -> List[ReachNode]:
        """
        Splits a reachable set w.r.t lanelet regions.

        Steps:
            1. Intersect reachable set in the position domain with lanelet regions
            2. Over-approximate and restore to axis-aligned rectangles
        """
        list_sets_split = []
        # iterate through regions intersecting with the reachable set
        for region in self.semantic_model.region_model.list_regions:
            # first compute intersection with bounding box
            # --> exact intersection is more expensive, so we only want to compute it if necessary?
            # there is no possibility of intersection
            if not region.intersects(reachable_set.position_rectangle.bounds, coordinate_system="CVLN"):
                continue

            # there is a possibility of intersection
            # TODO: Find out, why there was a try-except for Exception here
            polygon_intersection = region.polygon_cvln.intersection(reachable_set.position_rectangle)

            # empty intersection
            if not polygon_intersection or polygon_intersection.is_empty:
                continue

            # over-approximate by restoring the intersected polygon to axis-aligned rectangle
            bounds_polygon_intersection = polygon_intersection.bounds

            # clone the propagated set and split in the position domain, update the propositions
            # TODO: Find out, why there was a try-except for AttributeError here
            reachable_set_new = reachable_set.clone()
            self.reachable_set_to_propositions[reachable_set_new] = self.reachable_set_to_propositions[
                reachable_set].clone()
            reachable_set_new.intersect_in_position_domain(*bounds_polygon_intersection)
            reachable_set_new = self._update_propositions_with_region(reachable_set_new, region, step)

            list_sets_split.append(reachable_set_new)

        return list_sets_split

    def _update_propositions_with_region(self, propagated_set: Union[ReachNode, pycrreachs.SemanticReachNode],
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

        for group, set_propositions in dict_relevant.items():
            self.reachable_set_to_propositions[propagated_set].add_propositions(set_propositions, group)

        # add lanelet ids of the region to propagated set
        self.reachable_set_to_lanelet_ids[propagated_set] = frozenset(region.set_ids_lanelets)

        # add lanelet transition as temporary propositions
        set_propositions = self._obtain_lanelet_transition_propositions(propagated_set)
        self.reachable_set_to_propositions[propagated_set].add_propositions(set_propositions, PropGroup.TEMPORARY)

        return propagated_set

    def split_wrt_position_intervals(self, step: int,
                                     reachable_set: ReachNode) -> List[ReachNode]:
        """
        Splits the reachable set w.r.t position intervals.
        """

        list_intervals_lon = self.semantic_model.vehicle_model.dict_step_to_position_intervals[step]["lon"]
        list_intervals_lat = self.semantic_model.vehicle_model.dict_step_to_position_intervals[step]["lat"]

        list_reachable_sets_split_lon = self._split_reachable_set_wrt_intervals(reachable_set, list_intervals_lon,
                                                                                reachable_set.p_lon_min,
                                                                                reachable_set.p_lon_max, "lon")

        list_reachable_sets_split = list(itertools.chain.from_iterable(
            self._split_reachable_set_wrt_intervals(reachable_set_split, list_intervals_lat,
                                                    reachable_set_split.p_lat_min,
                                                    reachable_set_split.p_lat_max, "lat")
            for reachable_set_split in list_reachable_sets_split_lon))

        return list_reachable_sets_split

    def _split_reachable_set_wrt_intervals(self, reachable_set: ReachNode, intervals: List[PositionInterval],
                                           reach_min: float, reach_max: float, direction: str) \
            -> List[ReachNode]:
        list_reachable_sets_split = []
        for interval in intervals:
            if interval.intersects(reach_min, reach_max):
                propagated_set_split = reach_operation.split_reach_node_to_interval(reachable_set, interval, direction)
                if propagated_set_split:
                    self.copy_labels(reachable_set, propagated_set_split)
                    self.reachable_set_to_propositions[propagated_set_split].add_propositions(interval.set_propositions,
                                                                                              PropGroup.POSITION)
                    list_reachable_sets_split.append(propagated_set_split)

            # early termination, since the rest of intervals will definitely not intersect with the reachable set
            elif interval.p_min > reach_max:
                break
        return list_reachable_sets_split

    def _obtain_lanelet_transition_propositions(self,
                                                propagated_set: Union[ReachNode, pycrreachs.SemanticReachNode]):
        """
        Returns the set of lanelet transition propositions.
        """
        set_propositions = set()
        # retrieve lanelet propositions from the source
        if isinstance(propagated_set, ReachNode):
            source_node = propagated_set.source_propagation
        else:
            source_node = propagated_set.vec_nodes_source[0]

        set_propositions_position_source = self.reachable_set_to_propositions[source_node].propositions_in_group(
            PropGroup.POSITION)
        set_ids_lanelets_source = {int(proposition.split("_")[1]) for proposition in set_propositions_position_source
                                   if Prop.in_lanelet() in proposition}

        # generate lanelet transition propositions
        for id_lanelet_source in set_ids_lanelets_source:
            for id_lanelet_base_set in self.reachable_set_to_lanelet_ids[propagated_set]:
                if id_lanelet_source != id_lanelet_base_set:
                    set_propositions.add(Prop.lanelet_transition(id_lanelet_source, id_lanelet_base_set))

        return set_propositions

    def discard_colliding_nodes(self, list_propagated_set: List[ReachNode]) -> List[ReachNode]:
        """
        Returns a list of propagated sets that do not collide with vehicles.
        """
        list_nodes_keep = []

        for propagated_set in list_propagated_set:
            colliding = False
            set_propositions = self.reachable_set_to_propositions[propagated_set].set_propositions
            for proposition in set_propositions:
                # check if it is aligned with and besides a vehicle
                if Prop.aligned_with() in proposition:
                    id_vehicle = int(proposition.split("_")[1][1:])

                    if Prop.beside(id_vehicle) in set_propositions:
                        colliding = True
                        break

            if not colliding:
                list_nodes_keep.append(propagated_set)

        return list_nodes_keep

    def copy_labels(self, source_reachable_set: ReachNode, *reachable_sets: ReachNode) -> None:
        """Copy the labels of source_reachable_set to every node in reachable_sets."""
        for reachable_set in reachable_sets:
            self.reachable_set_to_propositions[reachable_set] = self.reachable_set_to_propositions[
                source_reachable_set].clone()
            self.reachable_set_to_lanelet_ids[reachable_set] = self.reachable_set_to_lanelet_ids[source_reachable_set]
