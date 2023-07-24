import warnings
from typing import List, Optional, Set

from commonroad_reach.data_structure.reach.reach_node import ReachNode

import commonroad_reach_semantic.data_structure.reach.predicates.predicate as predicate
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.rule.proposition import Proposition as Prop


class InFrontOfObstaclePredicate(predicate.Predicate):

    def __init__(self, obstacle_id: int, negated: bool):
        super().__init__(negated)
        self.obstacle_id = obstacle_id

    def to_proposition(self) -> str:
        return Prop.in_front_of(self.obstacle_id)

    def _restrict_reach_node_mandatory(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel,
                                       _node_lanelet_ids: Optional[Set[int]] = None) -> List[ReachNode]:
        if vehicle_front := self._get_vehicle_front(step, semantic_model):
            reach_node.intersect_in_position_domain(p_lon_min=vehicle_front)
            return [reach_node]
        else:
            warnings.warn(f"No prediction for {self.obstacle_id} at step {step}, cannot restrict reach node.")
            return [reach_node]

    def _restrict_reach_node_forbidden(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel,
                                       _node_lanelet_ids: Optional[Set[int]] = None) -> List[ReachNode]:
        if vehicle_front := self._get_vehicle_front(step, semantic_model):
            reach_node.intersect_in_position_domain(p_lon_max=vehicle_front)
            return [reach_node]
        else:
            warnings.warn(f"No prediction for {self.obstacle_id} at step {step}, cannot restrict reach node.")
            return [reach_node]

    def _get_vehicle_front(self, step: int, semantic_model: SemanticModel) -> Optional[float]:
        if vehicle := semantic_model.vehicle_model.find_vehicle_by_id(self.obstacle_id):
            if vehicle.has_ref_prediction(step):
                return vehicle.p_lon_max_ref(step, semantic_model.config.vehicle.ego.length / 2)
            else:
                return None
        else:
            raise RuntimeError(f"Vehicle {self.obstacle_id} not found")
