from typing import List, Set

from commonroad_reach.data_structure.reach.reach_node import ReachNode

import commonroad_reach_semantic.data_structure.reach.predicates.predicate as predicate
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.rule.proposition import Proposition as Prop


class OnRightLanePredicate(predicate.Predicate):
    def __init__(self, negated: bool):
        super().__init__(negated)
        self.needs_lanelets = True

    def to_proposition(self) -> str:
        return Prop.on_right_lane()

    @predicate.needs_lanelets_set
    def _restrict_reach_node_mandatory(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel,
                                       node_lanelet_ids: Set[int]) -> List[ReachNode]:
        if not node_lanelet_ids.isdisjoint(semantic_model.lanelet_model.right_lane_lanelet_ids):
            return [reach_node]
        else:
            return []

    @predicate.needs_lanelets_set
    def _restrict_reach_node_forbidden(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel,
                                       node_lanelet_ids: Set[int]) -> List[ReachNode]:
        if node_lanelet_ids.isdisjoint(semantic_model.lanelet_model.right_lane_lanelet_ids):
            return [reach_node]
        else:
            return []
