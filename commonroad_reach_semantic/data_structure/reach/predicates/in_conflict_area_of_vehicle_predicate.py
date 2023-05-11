from typing import List, Optional, Set

from commonroad_reach.data_structure.reach.reach_node import ReachNode

import commonroad_reach_semantic.data_structure.reach.predicates.predicate as predicate
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.rule.proposition import Proposition as Prop


class InConflictAreaOfVehiclePredicate(predicate.Predicate):

    def __init__(self, vehicle_id: int, negated: bool):
        super().__init__(negated)
        self.vehicle_id = vehicle_id
        self.needs_lanelets = True

    def to_proposition(self) -> str:
        return Prop.in_conflict_with(self.vehicle_id)

    @predicate.needs_lanelets_set
    def _restrict_reach_node_mandatory(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel,
                                       node_lanelet_ids: Set[int]) -> List[ReachNode]:
        if vehicle_lanelet_ids := self._get_vehicle_lanelet_ids(semantic_model):
            intersecting_lanelet_ids = {
                intersecting
                for lanelet_id in vehicle_lanelet_ids
                for intersecting in
                semantic_model.lanelet_model.dict_id_lanelet_to_set_ids_lanelets_intersecting[lanelet_id]
            }
            return [reach_node] if node_lanelet_ids.intersection(intersecting_lanelet_ids) else []
        else:
            raise RuntimeError(f"Vehicle {self.vehicle_id} not found")

    @predicate.needs_lanelets_set
    def _restrict_reach_node_forbidden(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel,
                                       node_lanelet_ids: Set[int]) -> List[ReachNode]:
        if vehicle_lanelet_ids := self._get_vehicle_lanelet_ids(semantic_model):
            intersecting_lanelet_ids = {
                intersecting
                for lanelet_id in vehicle_lanelet_ids
                for intersecting in
                semantic_model.lanelet_model.dict_id_lanelet_to_set_ids_lanelets_intersecting[lanelet_id]
            }
            return [reach_node] if node_lanelet_ids.isdisjoint(intersecting_lanelet_ids) else []
        else:
            raise RuntimeError(f"Vehicle {self.vehicle_id} not found")

    def _get_vehicle_lanelet_ids(self, semantic_model: SemanticModel) -> Optional[Set[int]]:
        if vehicle := semantic_model.vehicle_model.find_vehicle_by_id(self.vehicle_id):
            return vehicle.lane.list_ids_lanelets
        else:
            return None
