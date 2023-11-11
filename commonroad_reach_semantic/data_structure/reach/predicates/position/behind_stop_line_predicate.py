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
        stop_line_lanelet_ids = semantic_model.lanelet_model.stop_line_lanelet_ids

        # Return empty list if no stop line lanelet IDs exist
        if not stop_line_lanelet_ids:
            return []

        # Find lanelet IDs that are both in stop_line_lanelet_ids and node_lanelet_ids
        occupied_lanelet_ids = set(stop_line_lanelet_ids) & set(node_lanelet_ids or [])

        # Return empty list if no occupied lanelets with stop lines are found
        if not occupied_lanelet_ids:
            return []

        for lanelet_id in occupied_lanelet_ids:
            stop_line_s = self._get_stop_line(semantic_model, lanelet_id)

            if stop_line_s is not None:
                # Calculate positions relative to the stop line
                dis_stop_line = semantic_model.config.traffic_rule.dis_stop_line
                vehicle_length_half = semantic_model.config.vehicle.ego.length / 2

                min_position = stop_line_s - dis_stop_line - vehicle_length_half
                max_position = stop_line_s - vehicle_length_half

                # Adjust the reach node positions
                reach_node.intersect_in_position_domain(
                    p_lon_min=min_position, p_lon_max=max_position
                )

        return [reach_node.clone()] if reach_node else []

    def _restrict_reach_node_forbidden(
        self,
        step: int,
        reach_node: ReachNode,
        semantic_model: SemanticModel,
        node_lanelet_ids: Optional[Set[int]] = None,
    ) -> List[ReachNode]:
        stop_line_lanelet_ids = semantic_model.lanelet_model.stop_line_lanelet_ids

        # Return early if no stop line lanelet IDs are found
        if not stop_line_lanelet_ids:
            return [reach_node]

        # Find lanelet IDs that are both in stop_line_lanelet_ids and node_lanelet_ids
        occupied_lanelet_ids = set(stop_line_lanelet_ids) & set(node_lanelet_ids or [])

        # Return early if no occupied lanelets with stop lines are found
        if not occupied_lanelet_ids:
            return [reach_node]

        result_nodes = []
        for lanelet_id in occupied_lanelet_ids:
            stop_line_s = self._get_stop_line(semantic_model, lanelet_id)

            if stop_line_s is not None:
                # Calculate positions relative to the stop line
                dis_stop_line = semantic_model.config.traffic_rule.dis_stop_line
                vehicle_length_half = semantic_model.config.vehicle.ego.length / 2

                behind_stop_line_position = (
                    stop_line_s - dis_stop_line - vehicle_length_half
                )
                in_front_stop_line_position = stop_line_s - vehicle_length_half

                # Clone and adjust the reach node positions
                behind_sl_node = reach_node.clone()
                behind_sl_node.intersect_in_position_domain(
                    p_lon_max=behind_stop_line_position
                )

                in_front_sl_node = reach_node.clone()
                in_front_sl_node.intersect_in_position_domain(
                    p_lon_min=in_front_stop_line_position
                )

                result_nodes.extend([behind_sl_node, in_front_sl_node])

        return result_nodes if result_nodes else [reach_node]

    @staticmethod
    def _get_stop_line(
        semantic_model: SemanticModel, lanelet_with_sl_id: int
    ) -> Optional[float]:
        """Gets the longitudinal coordinate of the stop line."""

        lanelet = semantic_model.config.scenario.lanelet_network.find_lanelet_by_id(
            lanelet_with_sl_id
        )
        stop_line = lanelet.stop_line
        convert_coords = (
            semantic_model.config.planning.CLCS.convert_to_curvilinear_coords
        )

        start_s = convert_coords(*stop_line.start)[0]
        end_s = convert_coords(*stop_line.end)[0]

        return min(start_s, end_s) if start_s and end_s else None
