from typing import List, Optional, Set

from commonroad_reach.data_structure.reach.reach_node import ReachNode

import commonroad_reach_semantic.data_structure.reach.predicates.predicate as predicate
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.rule.proposition import Proposition as Prop


class InSameLanePredicate(predicate.Predicate):
    def __init__(self, vehicle_id: int, negated: bool):
        super().__init__(negated)
        self.vehicle_id = vehicle_id
        self.needs_lanelets = True

    def to_proposition(self) -> str:
        return Prop.in_same_lane(self.vehicle_id)

    @predicate.needs_lanelets_set
    def _restrict_reach_node_mandatory(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel,
                                       node_lanelet_ids: Set[int]) -> List[ReachNode]:
        vehicle = semantic_model.vehicle_model.find_vehicle_by_id(self.vehicle_id)
        other_lanelet_ids = vehicle.lanelet_ids_at_step(step)
        return [reach_node] if node_lanelet_ids.intersection(other_lanelet_ids) else []

    @predicate.needs_lanelets_set
    def _restrict_reach_node_forbidden(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel,
                                       node_lanelet_ids: Set[int]) -> List[ReachNode]:
        vehicle = semantic_model.vehicle_model.find_vehicle_by_id(self.vehicle_id)
        other_lanelet_ids = vehicle.lanelet_ids_at_step(step)
        return [] if node_lanelet_ids.intersection(other_lanelet_ids) else [reach_node]


