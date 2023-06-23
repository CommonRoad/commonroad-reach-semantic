from typing import List, Set, Optional

from commonroad_reach.data_structure.reach.reach_node import ReachNode

import commonroad_reach_semantic.data_structure.reach.predicates.predicate as predicate
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.rule.proposition import Proposition as Prop


class VehicleInLaneletPredicate(predicate.Predicate):
    """Indicates whether a vehicle is in the given lanelet."""

    def __init__(self, vehicle_id: int, lanelet_id: int, negated: bool):
        super().__init__(negated)
        self.vehicle_id = vehicle_id
        self.lanelet_id = lanelet_id

    def to_proposition(self) -> str:
        return Prop.vehicle_in_lanelet(self.vehicle_id, self.lanelet_id)

    def _restrict_reach_node_mandatory(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel,
                                       node_lanelet_ids: Optional[Set[int]] = None) -> List[ReachNode]:
        return [reach_node] if self._is_vehicle_in_lanelet(step, semantic_model) else []

    def _restrict_reach_node_forbidden(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel,
                                       node_lanelet_ids: Optional[Set[int]] = None) -> List[ReachNode]:
        return [reach_node] if not self._is_vehicle_in_lanelet(step, semantic_model) else []

    def _is_vehicle_in_lanelet(self, step: int, semantic_model: SemanticModel) -> bool:
        if vehicle := semantic_model.vehicle_model.find_vehicle_by_id(self.vehicle_id):
            return self.lanelet_id in vehicle.lanelet_ids_at_step(step)
        else:
            raise RuntimeError(f"Vehicle {self.vehicle_id} not found")
