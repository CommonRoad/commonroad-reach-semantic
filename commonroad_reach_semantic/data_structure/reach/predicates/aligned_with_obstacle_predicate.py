import warnings
from typing import List, Optional, Tuple, Set

from commonroad_reach.data_structure.reach.reach_node import ReachNode

import commonroad_reach_semantic.data_structure.reach.predicates.predicate as predicate
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.rule.proposition import Proposition as Prop


class AlignedWithObstaclePredicate(predicate.Predicate):

    def __init__(self, obstacle_id: int, negated: bool):
        super().__init__(negated)
        self.obstacle_id = obstacle_id

    def to_proposition(self) -> str:
        return Prop.beside(self.obstacle_id)

    def _restrict_reach_node_mandatory(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel,
                                       _node_lanelet_ids: Optional[Set[int]] = None) -> List[ReachNode]:
        if left_right := self._get_vehicle_left_right(step, semantic_model):
            vehicle_left, vehicle_right = left_right
            reach_node.intersect_in_position_domain(p_lat_min=vehicle_right, p_lat_max=vehicle_left)
            return [reach_node]
        else:
            warnings.warn(f"No prediction for {self.obstacle_id} at step {step}, cannot restrict reach node.")
            return [reach_node]

    def _restrict_reach_node_forbidden(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel,
                                       _node_lanelet_ids: Optional[Set[int]] = None) -> List[ReachNode]:
        if front_rear := self._get_vehicle_left_right(step, semantic_model):
            vehicle_left, vehicle_right = front_rear
            right = reach_node.clone()
            right.intersect_in_position_domain(p_lat_max=vehicle_right)
            left = reach_node  # reuse old reach node
            left.intersect_in_position_domain(p_lat_min=vehicle_left)
            return [right, left]
        else:
            warnings.warn(f"No prediction for {self.obstacle_id} at step {step}, cannot restrict reach node.")
            return [reach_node]

    def _get_vehicle_left_right(self, step: int, semantic_model: SemanticModel) -> Optional[Tuple[float, float]]:
        if vehicle := semantic_model.vehicle_model.find_vehicle_by_id(self.obstacle_id):
            if vehicle.has_ref_prediction(step):
                vehicle_left = vehicle.p_lat_max_ref(step, semantic_model.config.vehicle.ego.width / 2)
                vehicle_right = vehicle.p_lat_min_ref(step, semantic_model.config.vehicle.ego.width / 2)
                return vehicle_left, vehicle_right
            else:
                return None
        else:
            raise RuntimeError(f"Vehicle {self.obstacle_id} not found")
