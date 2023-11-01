from typing import List, Optional, Set

from commonroad_reach.data_structure.reach.reach_node import ReachNode

import commonroad_reach_semantic.data_structure.reach.predicates.predicate as predicate
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import (
    SemanticModel,
)
from commonroad_reach_semantic.data_structure.rule.proposition import (
    Proposition as Prop,
)


class BehindStopLinePredicate(predicate.Predicate):
    def __init__(self, negated: bool):
        super().__init__(negated)
        self.needs_lanelets = True

    def to_proposition(self) -> str:
        return Prop.behind_stop_line()

    def _restrict_reach_node_mandatory(
        self,
        step: int,
        reach_node: ReachNode,
        semantic_model: SemanticModel,
        node_lanelet_ids: Optional[Set[int]] = None,
    ) -> List[ReachNode]:
        lanelet_ids = semantic_model.lanelet_model.stop_line_lanelet_ids
        lanelet_with_sl_ids = node_lanelet_ids.intersection(lanelet_ids)
        if not lanelet_with_sl_ids:
            return []
        else:
            stop_line_s = self._get_stop_line(semantic_model, lanelet_with_sl_ids)
            reach_node.intersect_in_position_domain(p_lon_max=stop_line_s)
            return [reach_node]

    def _restrict_reach_node_forbidden(
        self,
        step: int,
        reach_node: ReachNode,
        semantic_model: SemanticModel,
        node_lanelet_ids: Optional[Set[int]] = None,
    ) -> List[ReachNode]:
        lanelet_ids = semantic_model.lanelet_model.stop_line_lanelet_ids
        lanelet_with_sl_ids = node_lanelet_ids.intersection(lanelet_ids)
        if not lanelet_with_sl_ids:
            return [reach_node]
        else:
            stop_line_s = self._get_stop_line(semantic_model, lanelet_with_sl_ids)
            reach_node.intersect_in_position_domain(p_lat_min=stop_line_s)
            return [reach_node]

    @staticmethod
    def _get_stop_line(
        semantic_model: SemanticModel, lanelet_with_sl_ids: Set[int]
    ) -> Optional[float]:
        # the closest stop line
        stop_line_s = min(
            [
                min(
                    semantic_model.config.planning.CLCS.convert_to_curvilinear_coords(
                        *semantic_model.config.scenario.lanelet_network.find_lanelet_by_id(
                            l
                        ).stop_line.start
                    )[0],
                    semantic_model.config.planning.CLCS.convert_to_curvilinear_coords(
                        *semantic_model.config.scenario.lanelet_network.find_lanelet_by_id(
                            l
                        ).stop_line.end
                    )[0],
                )
                for l in lanelet_with_sl_ids
            ]
        )
        if stop_line_s:
            return float(stop_line_s) - semantic_model.config.traffic_rule.dis_stop_line
        else:
            return None
