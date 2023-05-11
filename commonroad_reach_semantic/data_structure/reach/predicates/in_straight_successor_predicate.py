from typing import List, Set, Optional

from commonroad_reach.data_structure.reach.reach_node import ReachNode

import commonroad_reach_semantic.data_structure.reach.predicates.predicate as predicate
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.rule.proposition import Proposition as Prop


class InStraightSuccessorPredicate(predicate.Predicate):

    def __init__(self, negated: bool):
        super().__init__(negated)
        self.needs_lanelets = True

    def to_proposition(self) -> str:
        return Prop.in_straight_successor()

    @predicate.needs_lanelets_set
    def _restrict_reach_node_mandatory(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel,
                                       node_lanelet_ids: Set[int]) -> List[ReachNode]:
        lanelet_ids = semantic_model.config.semantic_model.incoming_element_route.successors_straight
        return [reach_node] if node_lanelet_ids.intersection(lanelet_ids) else []

    @predicate.needs_lanelets_set
    def _restrict_reach_node_forbidden(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel,
                                       node_lanelet_ids: Set[int]) -> List[ReachNode]:
        lanelet_ids = semantic_model.config.semantic_model.incoming_element_route.successors_straight
        return [reach_node] if node_lanelet_ids.isdisjoint(lanelet_ids) else []
