from typing import List, Optional, Set

from commonroad_reach.data_structure.reach.reach_node import ReachNode

import commonroad_reach_semantic.data_structure.reach.predicates.predicate as predicate
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.rule.proposition import Proposition as Prop

from commonroad.scenario.traffic_sign_interpreter import TrafficSignInterpreter
from commonroad.scenario.traffic_sign import SupportedTrafficSignCountry


class KeepsLaneSpeedLimitPredicate(predicate.Predicate):
    def __init__(self, negated: bool):
        super().__init__(negated)
        self.needs_lanelets = True

    def to_proposition(self) -> str:
        return Prop.lane_speed_limit()

    def _restrict_reach_node_mandatory(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel,
                                       _node_lanelet_ids: Optional[Set[int]] = None) -> List[ReachNode]:
        v_limit = self._obtain_lane_speed_limit(semantic_model, _node_lanelet_ids)
        if v_limit is not None:
            reach_node.intersect_in_velocity_domain(v_lon_max=v_limit)
        return [reach_node]

    def _restrict_reach_node_forbidden(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel,
                                       _node_lanelet_ids: Optional[Set[int]] = None) -> List[ReachNode]:
        v_limit = self._obtain_lane_speed_limit(semantic_model, _node_lanelet_ids)
        if v_limit is not None:
            reach_node.intersect_in_velocity_domain(v_lon_min=v_limit)
        return [reach_node]

    @staticmethod
    def _obtain_lane_speed_limit(semantic_model: SemanticModel, lanelet_ids: Optional[Set[int]]) -> float:
        # todo: fix for the other countries
        ts_interpreter = TrafficSignInterpreter(
            SupportedTrafficSignCountry("DEU"), semantic_model.lanelet_model.local_lanelet_network
        )
        return ts_interpreter.speed_limit(frozenset(lanelet_ids))
