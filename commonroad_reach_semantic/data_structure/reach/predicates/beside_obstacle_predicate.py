from typing import List, Optional, Tuple

from commonroad_reach.data_structure.reach.reach_node import ReachNode

import commonroad_reach_semantic.data_structure.reach.predicates.predicate as predicate
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.rule.proposition import Proposition as Prop


class BesideObstaclePredicate(predicate.Predicate):

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
