from typing import Union, List

from commonroad_reach.pycrreach import ReachPolygon

from commonroad_reach_semantic import pycrreachs
from commonroad_reach_semantic.data_structure.environment_model.region import Region
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.reach.semantic_reach_node import SemanticReachNode
from commonroad_reach_semantic.data_structure.rule.proposition import Proposition as Prop
from commonroad_reach_semantic.data_structure.rule.proposition import PropositionGroup as PropGroup
from commonroad_reach_semantic.data_structure.rule.proposition_holder import PropositionHolder
from commonroad_reach_semantic.utility import reach_operation


class ReachableSetLabeler:
    """Splits reachable sets and labels the parts according to the semantic model."""
    semantic_model: SemanticModel

    def __init__(self, semantic_model: SemanticModel):
        self.semantic_model = semantic_model

    def label_initial_state(self, drivable_areas: List[ReachPolygon], reachable_sets: List[SemanticReachNode],
                            step_start: int) -> None:
        """
        Assigns proposition labels to initial reachable sets and drivable areas.
        """
        for drivable_area, reachable_set in zip(drivable_areas, reachable_sets):
            propositions = self._obtain_propositions_for_rectangle(drivable_area, step_start)
            reachable_set.proposition_holder.merge(propositions)
        self.label_traffic_propositions(step_start, reachable_sets)

    def _obtain_propositions_for_rectangle(self, rectangle: ReachPolygon, step: int) -> PropositionHolder:
        """
        Returns the propositions of the given rectangle.

        Intersects the rectangle with regions and position intervals.
        """
        proposition_holder = PropositionHolder()
        # retrieve propositions from the intersecting lanelet region
        for region in self.semantic_model.region_model.list_regions:
            if region.polygon_cvln.intersects(rectangle):
                for group, set_propositions in region.dict_group_to_propositions_at_step(step).items():
                    proposition_holder.add_propositions(set_propositions, group)
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

        return proposition_holder

    def label_traffic_propositions(self, step,
                                   list_propagated_sets: Union[
                                       List[SemanticReachNode], List[pycrreachs.SemanticReachNode]]) \
            -> Union[List[SemanticReachNode], List[pycrreachs.SemanticReachNode]]:
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
                                               List[SemanticReachNode], List[pycrreachs.SemanticReachNode]]) \
            -> Union[List[SemanticReachNode], List[pycrreachs.SemanticReachNode]]:
        """
        Labels propagated sets with traffic status propositions.
        """
        set_propositions = self.semantic_model.traffic_status_model.dict_step_to_traffic_status_propositions[step]
        for propagated_set in list_propagated_sets:
            propagated_set.proposition_holder.add_propositions(set_propositions, PropGroup.TRAFFIC_STATUS)

        return list_propagated_sets

    def _label_in_conflict_area_propositions(self, step,
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
            for vehicle in self.semantic_model.vehicle_model.list_vehicles:
                # if not vehicle.behind_node_at_step(step, base_set):
                #     continue

                # iterate through lanelet ids of the region
                for id_lanelet_propagated_set in propagated_set.set_ids_lanelets:
                    # iterate through lanelet ids of the lane of the vehicle
                    for id_lanelet_lane_vehicle in vehicle.lane.list_ids_lanelets:
                        # use cached results
                        if id_lanelet_lane_vehicle in \
                                self.semantic_model.lanelet_model.dict_id_lanelet_to_set_ids_lanelets_intersecting[
                                    id_lanelet_propagated_set]:
                            propagated_set.proposition_holder.add_propositions(
                                {Prop.in_conflict_with(vehicle.id_vehicle)},
                                PropGroup.TRAFFIC_STATUS)

        # examine if the vehicles are in conflict with the propagated set
        for propagated_set in list_propagated_sets:
            if isinstance(propagated_set, SemanticReachNode):
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

                # iterate through lanelet ids of the route
                for id_lanelet_route in self.semantic_model.config.planning.route.list_ids_lanelets:
                    # iterate through lanelet ids of the vehicle
                    for id_lanelet_vehicle in vehicle.lanelet_ids_at_step(step):
                        if id_lanelet_vehicle in \
                                self.semantic_model.lanelet_model.dict_id_lanelet_to_set_ids_lanelets_intersecting[
                                    id_lanelet_route]:
                            propagated_set.proposition_holder.add_propositions(
                                {Prop.in_conflict_by(vehicle.id_vehicle)},
                                PropGroup.VEHICLE)

        return list_propagated_sets

    def _label_causes_braking_propositions(self, step: int,
                                           list_propagated_sets: Union[
                                               List[SemanticReachNode], List[pycrreachs.SemanticReachNode]]) \
            -> Union[List[SemanticReachNode], List[pycrreachs.SemanticReachNode]]:
        """
        Labels propagated sets with propositions related to causes braking to other vehicles.
        """
        # only compute if there is an intersection
        if not self.semantic_model.config.semantic_model.incoming_element_route:
            return list_propagated_sets

        for propagated_set in list_propagated_sets:
            # todo: this should only be computed for vehicles entering an intersection from other directions
            for vehicle in self.semantic_model.vehicle_model.list_vehicles:
                if vehicle.braking_caused_by_node_at_step(step, propagated_set):
                    propagated_set.proposition_holder.add_propositions({Prop.causes_braking_for(vehicle.id_vehicle)},
                                                                       PropGroup.TRAFFIC_STATUS)

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
            if polygon_intersection.is_empty:
                continue

            # over-approximate by restoring the intersected polygon to axis-aligned rectangle
            bounds_polygon_intersection = polygon_intersection.bounds

            # clone the propagated set and split in the position domain, update the propositions
            # TODO: Find out, why there was a try-except for AttributeError here
            reachable_set_new = reachable_set.clone()
            reachable_set_new.intersect_in_position_domain(*bounds_polygon_intersection)
            reachable_set_new = self._update_propositions_with_region(reachable_set_new, region, step)

            list_sets_split.append(reachable_set_new)

        return list_sets_split

    def _update_propositions_with_region(self, propagated_set: Union[SemanticReachNode, pycrreachs.SemanticReachNode],
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
        set_propositions = self._obtain_lanelet_transition_propositions(propagated_set)
        propagated_set.proposition_holder.add_propositions(set_propositions, PropGroup.TEMPORARY)

        return propagated_set

    def split_wrt_position_intervals(self, step: int,
                                     reachable_set: SemanticReachNode) -> List[SemanticReachNode]:
        """
        Splits the reachable set w.r.t position intervals.
        """

        list_intervals_lon = self.semantic_model.vehicle_model.dict_step_to_position_intervals[step]["lon"]
        list_intervals_lat = self.semantic_model.vehicle_model.dict_step_to_position_intervals[step]["lat"]

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
    def _obtain_lanelet_transition_propositions(propagated_set: Union[SemanticReachNode, pycrreachs.SemanticReachNode]):
        """
        Returns the set of lanelet transition propositions.
        """
        set_propositions = set()
        # retrieve lanelet propositions from the source
        if isinstance(propagated_set, SemanticReachNode):
            set_propositions_position_source = \
                propagated_set.source_propagation.proposition_holder.propositions_in_group(group=PropGroup.POSITION)

        else:
            set_propositions_position_source = \
                propagated_set.vec_nodes_source[0].proposition_holder.propositions_in_group(PropGroup.POSITION)

        set_ids_lanelets_source = {int(proposition.split("_")[1]) for proposition in set_propositions_position_source
                                   if Prop.in_lanelet() in proposition}

        # generate lanelet transition propositions
        for id_lanelet_source in set_ids_lanelets_source:
            for id_lanelet_base_set in propagated_set.set_ids_lanelets:
                if id_lanelet_source != id_lanelet_base_set:
                    set_propositions.add(Prop.lanelet_transition(id_lanelet_source, id_lanelet_base_set))

        return set_propositions
