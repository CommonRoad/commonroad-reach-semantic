import itertools
import re
from abc import ABC, abstractmethod
from functools import lru_cache
from typing import List, Optional, Tuple, Set

import commonroad_reach.utility.coordinate_system as util_cosy
import numpy as np
from commonroad_dc import pycrccosy
from commonroad_reach.data_structure.reach.reach_node import ReachNode
from commonroad_reach.data_structure.reach.reach_polygon import ReachPolygon

from commonroad_reach_semantic.data_structure.environment_model.region import Region
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.rule.proposition import Proposition as Prop


class Predicate(ABC):
    @staticmethod
    def from_proposition(proposition: str):
        if matched := re.fullmatch(r"InLanelet_(\d+)", proposition):
            return InLaneletPredicate(int(matched.group(1)))
        elif matched := re.fullmatch(r"Behind_V(\d+)", proposition):
            return BehindObstaclePredicate(int(matched.group(1)))
        elif matched := re.fullmatch(r"Beside_V(\d+)", proposition):
            return BesideObstaclePredicate(int(matched.group(1)))
        elif matched := re.fullmatch(r"InFrontOf_V(\d+)", proposition):
            return InFrontOfObstaclePredicate(int(matched.group(1)))
        elif re.fullmatch(r"InStraightSuc", proposition):
            return InStraightSuccessorPredicate()
        elif re.fullmatch(r"InIntersection", proposition):
            return InIntersectionPredicate()
        elif matched := re.fullmatch(r"CausesBrakingFor_V(\d+)", proposition):
            return CausesBrakingPredicate(int(matched.group(1)))
        elif matched := re.fullmatch(r"InConflictWith_V(\d+)", proposition):
            return InConflictAreaOfVehiclePredicate(int(matched.group(1)))
        elif matched := re.fullmatch(r"InConflictBy_V(\d+)", proposition):
            return VehicleInConflictAreaPredicate(int(matched.group(1)))
        else:
            raise ValueError(f"Unknown proposition: {proposition}")

    @abstractmethod
    def to_proposition(self) -> str:
        pass

    def restrict_reach_node(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel, negated: bool) -> \
            List[ReachNode]:
        if negated:
            restricted_nodes = self.restrict_reach_node_forbidden(step, reach_node, semantic_model)
        else:
            restricted_nodes = self.restrict_reach_node_mandatory(step, reach_node, semantic_model)
        return [
            node for node in restricted_nodes if not node.is_empty
        ]

    @abstractmethod
    def restrict_reach_node_mandatory(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel) -> List[
        ReachNode]:
        pass

    @abstractmethod
    def restrict_reach_node_forbidden(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel) -> List[
        ReachNode]:
        pass

    @staticmethod
    def _cut_to_region(reach_node: ReachNode, region: Region) -> Optional[ReachNode]:
        if not region.intersects(reach_node.position_rectangle.bounds, coordinate_system="CVLN"):
            return None

        # there is a possibility of intersection
        polygon_intersection = region.polygon_cvln.intersection(reach_node.position_rectangle)

        # empty intersection
        if polygon_intersection.is_empty:
            return None

        # over-approximate by restoring the intersected polygon to axis-aligned rectangle
        bounds_polygon_intersection = polygon_intersection.bounds

        # clone the reach node and cut down to region in the position domain
        reach_node_new = reach_node.clone()
        reach_node_new.intersect_in_position_domain(*bounds_polygon_intersection)
        return reach_node_new


class InLaneletPredicate(Predicate):
    def __init__(self, lanelet_id: int):
        self.lanelet_id = lanelet_id

    def to_proposition(self) -> str:
        return Prop.in_lanelet(self.lanelet_id)

    def restrict_reach_node_mandatory(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel) -> List[
        ReachNode]:
        split_sets = []
        for region in semantic_model.region_model.list_regions:
            if self.lanelet_id in region.set_ids_lanelets:
                if reach_node_new := self._cut_to_region(reach_node, region):
                    split_sets.append(reach_node_new)
        return split_sets

    def restrict_reach_node_forbidden(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel) -> List[
        ReachNode]:
        split_sets = []
        for region in semantic_model.region_model.list_regions:
            if self.lanelet_id not in region.set_ids_lanelets:
                if reach_node_new := self._cut_to_region(reach_node, region):
                    split_sets.append(reach_node_new)
        return split_sets


class BehindObstaclePredicate(Predicate):
    def __init__(self, obstacle_id: int):
        self.obstacle_id = obstacle_id

    def to_proposition(self) -> str:
        return Prop.behind(self.obstacle_id)

    def restrict_reach_node_mandatory(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel) -> List[
        ReachNode]:
        if vehicle_rear := self._get_vehicle_rear(step, semantic_model):
            reach_node.intersect_in_position_domain(p_lon_max=vehicle_rear)
            return [reach_node]
        else:
            raise RuntimeError(f"Vehicle with id {self.obstacle_id} not found")

    def restrict_reach_node_forbidden(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel) -> List[
        ReachNode]:
        if vehicle_rear := self._get_vehicle_rear(step, semantic_model):
            reach_node.intersect_in_position_domain(p_lon_min=vehicle_rear)
            return [reach_node]
        else:
            raise RuntimeError(f"Vehicle with id {self.obstacle_id} not found")

    def _get_vehicle_rear(self, step: int, semantic_model: SemanticModel) -> Optional[float]:
        if vehicle := semantic_model.vehicle_model.find_vehicle_by_id(self.obstacle_id):
            return vehicle.p_lon_min_ref(step, semantic_model.config.vehicle.ego.length / 2)
        return None


class BesideObstaclePredicate(Predicate):

    def __init__(self, obstacle_id: int):
        self.obstacle_id = obstacle_id

    def to_proposition(self) -> str:
        return Prop.beside(self.obstacle_id)

    def restrict_reach_node_mandatory(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel) -> List[
        ReachNode]:
        if front_rear := self._get_vehicle_front_rear(step, semantic_model):
            vehicle_front, vehicle_rear = front_rear
            reach_node.intersect_in_position_domain(p_lon_min=vehicle_rear, p_lon_max=vehicle_front)
            return [reach_node]
        else:
            raise RuntimeError(f"Vehicle {self.obstacle_id} not found")

    def restrict_reach_node_forbidden(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel) -> List[
        ReachNode]:
        if front_rear := self._get_vehicle_front_rear(step, semantic_model):
            vehicle_front, vehicle_rear = front_rear
            behind = reach_node.clone()
            behind.intersect_in_position_domain(p_lon_max=vehicle_rear)
            in_front = reach_node  # reuse old reach node
            in_front.intersect_in_position_domain(p_lon_min=vehicle_front)
            return [behind, in_front]
        else:
            raise RuntimeError(f"Vehicle {self.obstacle_id} not found")

    def _get_vehicle_front_rear(self, step: int, semantic_model: SemanticModel) -> Optional[Tuple[float, float]]:
        if vehicle := semantic_model.vehicle_model.find_vehicle_by_id(self.obstacle_id):
            vehicle_front = vehicle.p_lon_max_ref(step, semantic_model.config.vehicle.ego.length / 2)
            vehicle_rear = vehicle.p_lon_min_ref(step, semantic_model.config.vehicle.ego.length / 2)
            return vehicle_front, vehicle_rear
        else:
            return None


class InFrontOfObstaclePredicate(Predicate):

    def __init__(self, obstacle_id: int):
        self.obstacle_id = obstacle_id

    def to_proposition(self) -> str:
        return Prop.in_front_of(self.obstacle_id)

    def restrict_reach_node_mandatory(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel) -> List[
        ReachNode]:
        if vehicle_front := self._get_vehicle_front(step, semantic_model):
            reach_node.intersect_in_position_domain(p_lon_min=vehicle_front)
            return [reach_node]
        else:
            raise RuntimeError(f"Vehicle {self.obstacle_id} not found")

    def restrict_reach_node_forbidden(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel) -> List[
        ReachNode]:
        if vehicle_front := self._get_vehicle_front(step, semantic_model):
            reach_node.intersect_in_position_domain(p_lon_max=vehicle_front)
            return [reach_node]
        else:
            raise RuntimeError(f"Vehicle {self.obstacle_id} not found")

    def _get_vehicle_front(self, step: int, semantic_model: SemanticModel) -> Optional[float]:
        if vehicle := semantic_model.vehicle_model.find_vehicle_by_id(self.obstacle_id):
            return vehicle.p_lon_max_ref(step, semantic_model.config.vehicle.ego.length / 2)
        else:
            return None


class InStraightSuccessorPredicate(Predicate):

    def to_proposition(self) -> str:
        return Prop.in_straight_successor()

    def restrict_reach_node_mandatory(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel) -> List[
        ReachNode]:
        lanelet_ids = semantic_model.config.semantic_model.incoming_element_route.successors_straight
        split_sets = []
        for region in semantic_model.region_model.list_regions:
            if region.set_ids_lanelets.intersection(lanelet_ids):
                if reach_node_new := self._cut_to_region(reach_node, region):
                    split_sets.append(reach_node_new)
        return split_sets

    def restrict_reach_node_forbidden(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel) -> List[
        ReachNode]:
        lanelet_ids = semantic_model.config.semantic_model.incoming_element_route.successors_straight
        split_sets = []
        for region in semantic_model.region_model.list_regions:
            if region.set_ids_lanelets.isdisjoint(lanelet_ids):
                if reach_node_new := self._cut_to_region(reach_node, region):
                    split_sets.append(reach_node_new)
        return split_sets


class InIntersectionPredicate(Predicate):

    def to_proposition(self) -> str:
        return Prop.in_intersection()

    def restrict_reach_node_mandatory(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel) -> List[
        ReachNode]:
        lanelet_ids = semantic_model.lanelet_model.set_ids_lanelets_in_intersections
        split_sets = []
        for region in semantic_model.region_model.list_regions:
            if region.set_ids_lanelets.intersection(lanelet_ids):
                if reach_node_new := self._cut_to_region(reach_node, region):
                    split_sets.append(reach_node_new)
        return split_sets

    def restrict_reach_node_forbidden(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel) -> List[
        ReachNode]:
        lanelet_ids = semantic_model.lanelet_model.set_ids_lanelets_in_intersections
        split_sets = []
        for region in semantic_model.region_model.list_regions:
            if region.set_ids_lanelets.isdisjoint(lanelet_ids):
                if reach_node_new := self._cut_to_region(reach_node, region):
                    split_sets.append(reach_node_new)
        return split_sets


class CausesBrakingPredicate(Predicate):

    def __init__(self, obstacle_id: int):
        self.obstacle_id = obstacle_id

    def to_proposition(self) -> str:
        return Prop.causes_braking_for(self.obstacle_id)

    def restrict_reach_node_mandatory(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel) -> List[
        ReachNode]:
        if vehicle := semantic_model.vehicle_model.find_vehicle_by_id(self.obstacle_id):
            # ensure small distance
            try:
                p_lon_ego_max_vehicle = vehicle.p_lon_ego(step) + vehicle.shape.length / 2
            except KeyError:
                # no prediction for vehicle at step, so we assume it is not present anymore/yet
                # thus, we cannot cause it to brake
                return []
            # small distance means we are closer to the vehicle than `distance_braking` in the rule config

            # calculate the minimal stopping distance of the vehicle (without braking harder than allowed)
            v_lon_ego_vehicle = vehicle.v_lon_ego(step)
            reaction_distance = v_lon_ego_vehicle * semantic_model.config.vehicle.other.t_react
            braking_distance = - (v_lon_ego_vehicle ** 2) / (2 * semantic_model.config.traffic_rule.acceleration_braking_hard)
            stopping_distance = reaction_distance + braking_distance

            # the reach node is close enough to cause hard braking if it is closer than
            # the stopping distance of the vehicle using maximum allowed braking acceleration OR
            # the distance defined by the traffic rule
            max_lon_distance = max(stopping_distance, semantic_model.config.traffic_rule.distance_braking)
            p_lon_ego_max_reach_node = p_lon_ego_max_vehicle + max_lon_distance + \
                semantic_model.config.vehicle.ego.radius_inflation

            # ensure reach node is in front of vehicle
            # adding the vehicle length/2 is copied from vehicle.py without me fully understanding what it achieves
            p_lon_ego_min_reach_node = p_lon_ego_max_vehicle + \
                semantic_model.config.vehicle.other.length / 2 - \
                semantic_model.config.vehicle.ego.radius_inflation

            # transform position rectangle of the reach node to the CLCS of the vehicle
            try:
                cart_position_rect = self._convert_to_cartesian_vertices(reach_node.position_rectangle.vertices, vehicle.CLCS_ref)
            except ValueError:
                # when the reach node is outside the projection domain of the vehicle's CLCS, we the reach node
                # is far away, so it cannot cause braking
                return []
            ego_position_rect = ReachPolygon(util_cosy.convert_to_curvilinear_vertices(cart_position_rect, vehicle.lane.CLCS))

            # intersect with halfspaces (if an intersection is empty, we drop the reach node)
            intersected = ego_position_rect.intersect_halfspace(1, 0, p_lon_ego_max_reach_node)
            if intersected is None:
                return []
            intersected = intersected.intersect_halfspace(-1, 0, -p_lon_ego_min_reach_node)
            if intersected is None:
                return []

            # transform result back to the CLCS of the reach node and restore rectangle shape
            cart_intersected = self._convert_to_cartesian_vertices(intersected.vertices, vehicle.lane.CLCS)
            ref_intersected = ReachPolygon(util_cosy.convert_to_curvilinear_vertices(cart_intersected, vehicle.CLCS_ref))
            reach_node.intersect_in_position_domain(*ref_intersected.bounds)

            return [reach_node]
        else:
            raise RuntimeError(f"Vehicle {self.obstacle_id} not found")

    def restrict_reach_node_forbidden(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel) -> List[
        ReachNode]:
        if vehicle := semantic_model.vehicle_model.find_vehicle_by_id(self.obstacle_id):
            # ensure small distance
            try:
                p_lon_ego_max_vehicle = vehicle.p_lon_ego(step) + vehicle.shape.length / 2
            except KeyError:
                # no prediction for vehicle at step, so we assume it is not present anymore/yet
                # thus, we cannot cause it to brake
                return [reach_node]
            # small distance means we are closer to the vehicle than `distance_braking` in the rule config

            # calculate the minimal stopping distance of the vehicle (without braking harder than allowed)
            v_lon_ego_vehicle = vehicle.v_lon_ego(step)
            reaction_distance = v_lon_ego_vehicle * semantic_model.config.vehicle.other.t_react
            braking_distance = - (v_lon_ego_vehicle ** 2) / (
                        2 * semantic_model.config.traffic_rule.acceleration_braking_hard)
            stopping_distance = reaction_distance + braking_distance

            # the reach node is close enough to cause hard braking if it is closer than
            # the stopping distance of the vehicle using maximum allowed braking acceleration OR
            # the distance defined by the traffic rule
            max_lon_distance = max(stopping_distance, semantic_model.config.traffic_rule.distance_braking)
            p_lon_ego_max_reach_node = p_lon_ego_max_vehicle + max_lon_distance + \
                                       semantic_model.config.vehicle.ego.radius_inflation

            # ensure reach node is in front of vehicle
            # adding the vehicle length/2 is copied from vehicle.py without me fully understanding what it achieves
            p_lon_ego_min_reach_node = p_lon_ego_max_vehicle + \
                                       semantic_model.config.vehicle.other.length / 2 - \
                                       semantic_model.config.vehicle.ego.radius_inflation

            # transform position rectangle of the reach node to the CLCS of the vehicle
            try:
                cart_position_rect = self._convert_to_cartesian_vertices(reach_node.position_rectangle.vertices,
                                                                         vehicle.CLCS_ref)
            except ValueError:
                # when the reach node is outside the projection domain of the vehicle's CLCS, we the reach node
                # is far away, so it cannot cause braking
                return [reach_node]
            ego_position_rect = ReachPolygon(
                util_cosy.convert_to_curvilinear_vertices(cart_position_rect, vehicle.lane.CLCS))

            # intersect with halfspaces (if an intersection is empty, we drop the reach node)
            after = ego_position_rect.intersect_halfspace(-1, 0, -p_lon_ego_max_reach_node)
            before = ego_position_rect.intersect_halfspace(1, 0, p_lon_ego_min_reach_node)

            # transform result back to the CLCS of the reach node and restore rectangle shape
            new_nodes = []
            if after is not None:
                cart_after = self._convert_to_cartesian_vertices(after.vertices, vehicle.lane.CLCS)
                ref_after = ReachPolygon(
                    util_cosy.convert_to_curvilinear_vertices(cart_after, vehicle.CLCS_ref))
                after_node = reach_node.clone()
                after_node.intersect_in_position_domain(*ref_after.bounds)
                new_nodes.append(after_node)
            if before is not None:
                cart_before = self._convert_to_cartesian_vertices(before.vertices, vehicle.lane.CLCS)
                ref_before = ReachPolygon(
                    util_cosy.convert_to_curvilinear_vertices(cart_before, vehicle.CLCS_ref))
                reach_node.intersect_in_position_domain(*ref_before.bounds)  # Reuse reach node
                new_nodes.append(reach_node)

            return new_nodes
        else:
            raise RuntimeError(f"Vehicle {self.obstacle_id} not found")

    def _convert_to_cartesian_vertices(self, vertices_cvln: np.ndarray, CLCS: pycrccosy.CurvilinearCoordinateSystem):
        """
        Converts a list of Curvilinear vertices to Cartesian vertices.
        """
        list_vertices_cart = [CLCS.convert_to_cartesian_coords(vertex[0], vertex[1]) for vertex in vertices_cvln]
        return list_vertices_cart

class InConflictAreaOfVehiclePredicate(Predicate):

    def __init__(self, vehicle_id: int):
        self.vehicle_id = vehicle_id

    def to_proposition(self) -> str:
        return Prop.in_conflict_with(self.vehicle_id)

    def restrict_reach_node_mandatory(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel) -> List[
        ReachNode]:
        if vehicle_lanelet_ids := self._get_vehicle_lanelet_ids(semantic_model):
            intersecting_lanelet_ids = {
                intersecting
                for lanelet_id in vehicle_lanelet_ids
                for intersecting in
                semantic_model.lanelet_model.dict_id_lanelet_to_set_ids_lanelets_intersecting[lanelet_id]
            }
            split_sets = []
            for region in semantic_model.region_model.list_regions:
                if region.set_ids_lanelets.intersection(intersecting_lanelet_ids):
                    if reach_node_new := self._cut_to_region(reach_node, region):
                        split_sets.append(reach_node_new)
            return split_sets
        else:
            raise RuntimeError(f"Vehicle {self.vehicle_id} not found")

    def restrict_reach_node_forbidden(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel) -> List[
        ReachNode]:
        if vehicle_lanelet_ids := self._get_vehicle_lanelet_ids(semantic_model):
            intersecting_lanelet_ids = {
                intersecting
                for lanelet_id in vehicle_lanelet_ids
                for intersecting in
                semantic_model.lanelet_model.dict_id_lanelet_to_set_ids_lanelets_intersecting[lanelet_id]
            }
            split_sets = []
            for region in semantic_model.region_model.list_regions:
                if region.set_ids_lanelets.isdisjoint(intersecting_lanelet_ids):
                    if reach_node_new := self._cut_to_region(reach_node, region):
                        split_sets.append(reach_node_new)
            return split_sets
        else:
            raise RuntimeError(f"Vehicle {self.vehicle_id} not found")

    def _get_vehicle_lanelet_ids(self, semantic_model: SemanticModel) -> Optional[Set[int]]:
        if vehicle := semantic_model.vehicle_model.find_vehicle_by_id(self.vehicle_id):
            return vehicle.lane.list_ids_lanelets
        else:
            return None


class VehicleInConflictAreaPredicate(Predicate):

    def __init__(self, vehicle_id: int):
        self.vehicle_id = vehicle_id

    def to_proposition(self) -> str:
        return Prop.in_conflict_by(self.vehicle_id)

    def restrict_reach_node_mandatory(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel) -> List[
        ReachNode]:
        p_lon_max = self._p_lon_max_for_conflict_at_step(step, semantic_model)
        # If the minimal longitudinal position of the reach node is greater than the maximum,
        # it lies entirely in front of the vehicle --> discard the reach node as no conflict occurs
        if p_lon_max < reach_node.p_lon_min:
            return []

        # otherwise, check if the current vehicle position intersects with the route
        if self._vehicle_intersects_with_route(step, semantic_model):
            # we have a conflict, so cut off the part that lies in front of the vehicle and return the reach node
            reach_node.intersect_in_position_domain(p_lon_max=p_lon_max)
            return [reach_node]
        else:
            # there is no conflict, so discard the reach node
            return []

    def restrict_reach_node_forbidden(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel) -> List[
        ReachNode]:
        p_lon_max = self._p_lon_max_for_conflict_at_step(step, semantic_model)
        # If the minimal longitudinal position of the reach node is greater than the maximum,
        # it lies entirely in front of the vehicle --> keep the reach node unchanged as no conflict occurs
        if p_lon_max < reach_node.p_lon_min:
            return [reach_node]

        # otherwise, check if the current vehicle position intersects with the route
        if self._vehicle_intersects_with_route(step, semantic_model):
            # we have a conflict, so keep only the part of the reach node that is in front of the vehicle
            reach_node.intersect_in_position_domain(p_lon_min=p_lon_max)
            return [reach_node]
        else:
            # there is no conflict, so return the reach node unchanged
            return [reach_node]

    def _p_lon_max_for_conflict_at_step(self, step: int, semantic_model: SemanticModel) -> float:
        if vehicle := semantic_model.vehicle_model.find_vehicle_by_id(self.vehicle_id):
            # The reach node must not be in front of the vehicle along the reference path for the vehicle to cause a conflict
            # Thus, the maximal longitudinal position of our reach node must be smaller than
            # the longitudinal position of the vehicle + half the length of the vehicle (vehicle shape) + inflation (ego shape)
            p_lon_ref_max_vehicle = vehicle.p_lon_ref(step) + vehicle.shape.length / 2
            return p_lon_ref_max_vehicle + semantic_model.config.vehicle.ego.radius_inflation
        else:
            raise RuntimeError(f"Vehicle {self.vehicle_id} not found")

    @lru_cache(maxsize=None)
    def _vehicle_intersects_with_route(self, step: int, semantic_model: SemanticModel) -> bool:
        if vehicle := semantic_model.vehicle_model.find_vehicle_by_id(self.vehicle_id):
            vehicle_lanelets = vehicle.lanelet_ids_at_step(step)
            route_lanelets = semantic_model.config.planning.route.list_ids_lanelets
            for l_route, l_vehicle in itertools.product(route_lanelets, vehicle_lanelets):
                intersecting = semantic_model.lanelet_model.dict_id_lanelet_to_set_ids_lanelets_intersecting[l_route]
                if l_vehicle in intersecting:
                    # if one of the lanelets the vehicle currently occupies intersects with the route we have a conflict
                    return True
            return False
        else:
            raise RuntimeError(f"Vehicle {self.vehicle_id} not found")
