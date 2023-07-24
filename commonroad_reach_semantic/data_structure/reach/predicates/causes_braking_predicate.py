from typing import List, Optional, Tuple, Set

import commonroad_reach.utility.coordinate_system as util_cosy
import numpy as np
from commonroad_dc import pycrccosy
from commonroad_reach.data_structure.reach.reach_node import ReachNode
from commonroad_reach.data_structure.reach.reach_polygon import ReachPolygon

import commonroad_reach_semantic.data_structure.reach.predicates.predicate as predicate
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.environment_model.vehicle import Vehicle
from commonroad_reach_semantic.data_structure.rule.proposition import Proposition as Prop


class CausesBrakingPredicate(predicate.Predicate):

    def __init__(self, obstacle_id: int, negated: bool):
        super().__init__(negated)
        self.obstacle_id = obstacle_id

    def to_proposition(self) -> str:
        return Prop.causes_braking_for(self.obstacle_id)

    def _restrict_reach_node_mandatory(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel,
                                       _node_lanelet_ids: Optional[Set[int]] = None) -> List[ReachNode]:
        if vehicle := semantic_model.vehicle_model.find_vehicle_by_id(self.obstacle_id):
            if p_lon_min_max_ego := self._get_min_max_lon_to_cause_braking(step, semantic_model, vehicle):
                p_lon_ego_min, p_lon_ego_max = p_lon_min_max_ego
            else:
                # vehicle is not present anymore/yet, so we cannot cause braking
                return []

            ego_position_rect = self._position_rect_in_ego_cosy(reach_node, vehicle)
            if ego_position_rect is None:
                # when the reach node is outside the projection domain of the vehicle's CLCS, we assume the reach node
                # is too far away to cause braking
                return []

            # intersect with halfspaces (if an intersection is empty, we drop the reach node)
            intersected = ego_position_rect.intersect_halfspace(1, 0, p_lon_ego_max)
            if intersected is None:
                return []
            intersected = intersected.intersect_halfspace(-1, 0, -p_lon_ego_min)
            if intersected is None:
                return []

            # cut reach node to position intersection (uses .bounds to restore rectangular shape)
            reach_node.intersect_in_position_domain(*self._transform_polygon_to_ref_cosy(intersected, vehicle).bounds)
            return [reach_node]
        else:
            raise RuntimeError(f"Vehicle {self.obstacle_id} not found")

    def _restrict_reach_node_forbidden(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel,
                                       _node_lanelet_ids: Optional[Set[int]] = None) -> List[ReachNode]:
        if vehicle := semantic_model.vehicle_model.find_vehicle_by_id(self.obstacle_id):
            if p_lon_min_max_ego := self._get_min_max_lon_to_cause_braking(step, semantic_model, vehicle):
                p_lon_ego_min, p_lon_ego_max = p_lon_min_max_ego
            else:
                # vehicle is not present anymore/yet, so we cannot cause braking
                return [reach_node]

            ego_position_rect = self._position_rect_in_ego_cosy(reach_node, vehicle)
            if ego_position_rect is None:
                # when the reach node is outside the projection domain of the vehicle's CLCS, we assume the reach node
                # is far enough away to not cause braking
                return [reach_node]

            # intersect with halfspaces
            far_enough_away = ego_position_rect.intersect_halfspace(-1, 0, -p_lon_ego_max)
            not_in_front_of_vehicle = ego_position_rect.intersect_halfspace(1, 0, p_lon_ego_min)

            # transform result back to the CLCS of the reach node and restore rectangle shape
            new_nodes = []
            intersected_ego_position_rects = [
                rect for rect in (far_enough_away, not_in_front_of_vehicle) if rect is not None
            ]
            for i, intersected_ego_position_rect in enumerate(intersected_ego_position_rects):
                # Reuse the old reach node in the last iteration
                new_node = reach_node if i == len(intersected_ego_position_rects) - 1 else reach_node.clone()
                new_node.intersect_in_position_domain(
                    *self._transform_polygon_to_ref_cosy(intersected_ego_position_rect, vehicle).bounds)
                new_nodes.append(new_node)

            return new_nodes
        else:
            raise RuntimeError(f"Vehicle {self.obstacle_id} not found")

    @staticmethod
    def _get_min_max_lon_to_cause_braking(step: int, semantic_model: SemanticModel, vehicle: Vehicle) -> Optional[
        Tuple[float, float]]:
        """Calculates the minimum and maximum longitudinal position so that braking is caused when in between.

        The minimum position is chosen to ensure that the reach node is in front of the vehicle (otherwise it cannot cause braking).
        The maximum position is chosen to ensure that the reach node is close enough to the vehicle to cause hard braking.
        "Close enough" is defined by braking distance in the traffic rule or the stopping distance of the vehicle (using maximal allowed deceleration), whichever is larger.

        All coordinates are in the curvilinear coordinate system of the vehicle.

        :returns: (minimum_position, maximum_position) or None if the vehicle is not present at the given step
        """
        if not vehicle.has_ego_prediction(step):
            # no prediction for vehicle at step, so we assume it is not present anymore/yet
            return None
        p_lon_ego_max_vehicle = vehicle.p_lon_ego(step) + vehicle.shape.length / 2

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
        return p_lon_ego_min_reach_node, p_lon_ego_max_reach_node

    def _position_rect_in_ego_cosy(self, reach_node: ReachNode, vehicle: Vehicle) -> Optional[ReachPolygon]:
        """Transforms the position rectangle of the reach node to the CLCS of the vehicle.

        :returns: The transformed position rectangle or None if the reach node is outside the projection domain of the vehicle's CLCS
        """
        # transform position rectangle of the reach node to the CLCS of the vehicle
        try:
            cart_vertices = self._convert_to_cartesian_vertices(reach_node.position_rectangle.vertices,
                                                                vehicle.CLCS_ref)
        except ValueError:
            # reach node is outside the projection domain of the vehicle's CLCS
            return None
        return ReachPolygon(util_cosy.convert_to_curvilinear_vertices(cart_vertices, vehicle.lane.CLCS))

    def _transform_polygon_to_ref_cosy(self, polygon: ReachPolygon, vehicle: Vehicle) -> ReachPolygon:
        """Transforms a polygon from the CLCS of the vehicle to the CLCS of the reach node."""
        cart_vertices = self._convert_to_cartesian_vertices(polygon.vertices, vehicle.lane.CLCS)
        return ReachPolygon(util_cosy.convert_to_curvilinear_vertices(cart_vertices, vehicle.CLCS_ref))

    @staticmethod
    def _convert_to_cartesian_vertices(vertices_cvln: np.ndarray, CLCS: pycrccosy.CurvilinearCoordinateSystem):
        """
        Converts a list of Curvilinear vertices to Cartesian vertices.
        """
        list_vertices_cart = [CLCS.convert_to_cartesian_coords(vertex[0], vertex[1]) for vertex in vertices_cvln]
        return list_vertices_cart
