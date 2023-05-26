from typing import List, Optional, Set

from commonroad_reach.data_structure.reach.reach_node import ReachNode

import commonroad_reach_semantic.data_structure.reach.predicates.predicate as predicate
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.rule.proposition import Proposition as Prop


class RightOfObstaclePredicate(predicate.Predicate):
    def __init__(self, obstacle_id: int, negated: bool):
        super().__init__(negated)
        self.obstacle_id = obstacle_id

    def to_proposition(self) -> str:
        return Prop.right_of(self.obstacle_id)

    def _restrict_reach_node_mandatory(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel,
                                       _node_lanelet_ids: Optional[Set[int]] = None) -> List[ReachNode]:
        if vehicle_right := self._get_vehicle_right(step, semantic_model):
            reach_node.intersect_in_position_domain(p_lat_max=vehicle_right)
            return [reach_node]
        else:
            raise RuntimeError(f"Vehicle with id {self.obstacle_id} not found")

    def _restrict_reach_node_forbidden(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel,
                                       _node_lanelet_ids: Optional[Set[int]] = None) -> List[ReachNode]:
        if vehicle_right := self._get_vehicle_right(step, semantic_model):
            reach_node.intersect_in_position_domain(p_lat_min=vehicle_right)
            return [reach_node]
        else:
            raise RuntimeError(f"Vehicle with id {self.obstacle_id} not found")

    def _get_vehicle_right(self, step: int, semantic_model: SemanticModel) -> Optional[float]:
        if vehicle := semantic_model.vehicle_model.find_vehicle_by_id(self.obstacle_id):
            return vehicle.p_lat_min_ref(step, semantic_model.config.vehicle.ego.width / 2)
        return None
