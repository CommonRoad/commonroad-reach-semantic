from typing import List, Optional

from commonroad_reach.data_structure.reach.reach_node import ReachNode

import commonroad_reach_semantic.data_structure.reach.predicates.predicate as predicate
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.rule.proposition import Proposition as Prop


class BehindObstaclePredicate(predicate.Predicate):
    def __init__(self, obstacle_id: int, negated: bool):
        super().__init__(negated)
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
