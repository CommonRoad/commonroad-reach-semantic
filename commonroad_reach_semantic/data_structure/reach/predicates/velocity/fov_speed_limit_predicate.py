from typing import List, Optional, Set

from commonroad_reach.data_structure.reach.reach_node import ReachNode

import commonroad_reach_semantic.data_structure.reach.predicates.predicate as predicate
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.rule.proposition import Proposition as Prop


class FovSpeedLimitPredicate(predicate.Predicate):
    def __init__(self, negated: bool):
        super().__init__(negated)

    def to_proposition(self) -> str:
        return Prop.fov_speed_limit()

    def _restrict_reach_node_mandatory(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel,
                                       _node_lanelet_ids: Optional[Set[int]] = None) -> List[ReachNode]:
        v_limit = semantic_model.config.traffic_rule.fov_speed_limit
        if v_limit is not None:
            reach_node.intersect_in_velocity_domain(v_lon_max=v_limit)
        return [reach_node]

    def _restrict_reach_node_forbidden(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel,
                                       _node_lanelet_ids: Optional[Set[int]] = None) -> List[ReachNode]:
        v_limit = semantic_model.config.traffic_rule.fov_speed_limit
        if v_limit is not None:
            reach_node.intersect_in_velocity_domain(v_lon_min=v_limit)
        return [reach_node]