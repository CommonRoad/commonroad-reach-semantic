from typing import List, Optional, Set

from commonroad_reach.data_structure.reach.reach_node import ReachNode

import commonroad_reach_semantic.data_structure.reach.predicates.predicate as predicate
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.rule.proposition import Proposition as Prop


class InIntersectionPredicate(predicate.Predicate):

    def __init__(self, negated: bool):
        super().__init__(negated)
        self.needs_lanelets = True

    def to_proposition(self) -> str:
        return Prop.in_intersection()

    def _restrict_reach_node_mandatory(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel,
                                       node_lanelet_ids: Optional[Set[int]] = None) -> List[ReachNode]:
        lanelet_ids = semantic_model.lanelet_model.set_ids_lanelets_in_intersections
        return [reach_node] if node_lanelet_ids.intersection(lanelet_ids) else []

    def _restrict_reach_node_forbidden(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel,
                                       node_lanelet_ids: Optional[Set[int]] = None) -> List[ReachNode]:
        lanelet_ids = semantic_model.lanelet_model.set_ids_lanelets_in_intersections
        return [reach_node] if node_lanelet_ids.isdisjoint(lanelet_ids) else []
