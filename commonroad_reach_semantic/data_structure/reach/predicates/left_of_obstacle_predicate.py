from typing import List, Optional, Set

from commonroad_reach.data_structure.reach.reach_node import ReachNode

import commonroad_reach_semantic.data_structure.reach.predicates.predicate as predicate
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.rule.proposition import Proposition as Prop


class LeftOfObstaclePredicate(predicate.Predicate):

    def __init__(self, obstacle_id: int, negated: bool):
        super().__init__(negated)
        self.obstacle_id = obstacle_id

    def to_proposition(self) -> str:
        return Prop.in_front_of(self.obstacle_id)

    def _restrict_reach_node_mandatory(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel,
                                       _node_lanelet_ids: Optional[Set[int]] = None) -> List[ReachNode]:
        if vehicle_left := self._get_vehicle_left(step, semantic_model):
            reach_node.intersect_in_position_domain(p_lat_min=vehicle_left)
            return [reach_node]
        else:
            raise RuntimeError(f"Vehicle {self.obstacle_id} not found or no prediction for step {step}")

    def _restrict_reach_node_forbidden(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel,
                                       _node_lanelet_ids: Optional[Set[int]] = None) -> List[ReachNode]:
        if vehicle_left := self._get_vehicle_left(step, semantic_model):
            reach_node.intersect_in_position_domain(p_lat_max=vehicle_left)
            return [reach_node]
        else:
            raise RuntimeError(f"Vehicle {self.obstacle_id} not found or no prediction for step {step}")

    def _get_vehicle_left(self, step: int, semantic_model: SemanticModel) -> Optional[float]:
        if vehicle := semantic_model.vehicle_model.find_vehicle_by_id(self.obstacle_id):
            try:
                return vehicle.p_lat_max_ref(step, semantic_model.config.vehicle.ego.width / 2)
            except KeyError:
                return None
        else:
            return None
