import itertools
from functools import lru_cache
from typing import List

from commonroad_reach.data_structure.reach.reach_node import ReachNode

import commonroad_reach_semantic.data_structure.reach.predicates.predicate as predicate
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.rule.proposition import Proposition as Prop


class VehicleInConflictAreaPredicate(predicate.Predicate):

    def __init__(self, vehicle_id: int, negated: bool):
        super().__init__(negated)
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
