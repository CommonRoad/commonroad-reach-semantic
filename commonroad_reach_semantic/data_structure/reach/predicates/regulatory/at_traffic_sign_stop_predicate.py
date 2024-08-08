from typing import List, Set

from commonroad.scenario.traffic_sign import TrafficSignIDGermany

from commonroad_reach.data_structure.reach.reach_node import ReachNode
import commonroad_reach_semantic.data_structure.reach.predicates.predicate as predicate
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import (
    SemanticModel,
)
from commonroad_reach_semantic.data_structure.rule.proposition import (
    Proposition as Prop,
)


class AtTrafficSignStopPredicate(predicate.Predicate):
    def __init__(self, negated: bool):
        super().__init__(negated)
        self.needs_lanelets = True

    def to_proposition(self) -> str:
        return Prop.at_traffic_sign_stop()

    @predicate.needs_lanelets_set
    def _restrict_reach_node_mandatory(
        self,
        step: int,
        reach_node: ReachNode,
        semantic_model: SemanticModel,
        node_lanelet_ids: Set[int],
    ) -> List[ReachNode]:
        for lanelet_id in node_lanelet_ids:
            traffic_sign = semantic_model.lanelet_model.local_lanelet_network.find_traffic_sign_by_id(
                lanelet_id
            )
            if traffic_sign is None:
                continue

            if traffic_sign.traffic_sign_id == TrafficSignIDGermany.STOP:
                return [reach_node]
        return []

    @predicate.needs_lanelets_set
    def _restrict_reach_node_forbidden(
        self,
        step: int,
        reach_node: ReachNode,
        semantic_model: SemanticModel,
        node_lanelet_ids: Set[int],
    ) -> List[ReachNode]:
        for lanelet_id in node_lanelet_ids:
            traffic_sign = semantic_model.lanelet_model.local_lanelet_network.find_traffic_sign_by_id(
                lanelet_id
            )
            if traffic_sign is None:
                continue

            if traffic_sign.traffic_sign_id == TrafficSignIDGermany.STOP:
                return []
        return [reach_node]
