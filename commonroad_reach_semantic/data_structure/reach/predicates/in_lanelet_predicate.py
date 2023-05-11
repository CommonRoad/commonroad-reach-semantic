from typing import List, Optional, Set

from commonroad_reach.data_structure.reach.reach_node import ReachNode

import commonroad_reach_semantic.data_structure.reach.predicates.predicate as predicate
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.rule.proposition import Proposition as Prop


class InLaneletPredicate(predicate.Predicate):
    def __init__(self, lanelet_id: int, negated: bool):
        super().__init__(negated)
        self.lanelet_id = lanelet_id
        self.needs_lanelets = True

    def to_proposition(self) -> str:
        return Prop.in_lanelet(self.lanelet_id)

    @predicate.needs_lanelets_set
    def _restrict_reach_node_mandatory(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel,
                                       node_lanelet_ids: Set[int]) -> List[ReachNode]:
        return [reach_node] if self.lanelet_id in node_lanelet_ids else []

    @predicate.needs_lanelets_set
    def _restrict_reach_node_forbidden(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel,
                                       node_lanelet_ids: Set[int]) -> List[ReachNode]:
        return [reach_node] if self.lanelet_id not in node_lanelet_ids else []
