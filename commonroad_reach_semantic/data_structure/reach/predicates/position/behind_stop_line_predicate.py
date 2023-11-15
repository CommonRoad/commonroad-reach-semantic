from typing import List, Optional, Set

from commonroad_reach import pycrreach
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
        # Obtain the id of lanelets with stop lines
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
            # Obtain the longitudinal coordinate of the stop line
            stop_line_s = self._get_stop_line(semantic_model, lanelet_id)

            if stop_line_s is not None:
                # Calculate positions relative to the stop line
                dis_stop_line = semantic_model.config.traffic_rule.dis_stop_line

                # Additional consideration of the vehicle length
                vehicle_length_add = semantic_model.config.vehicle.ego.length / 2
                if semantic_model.config.planning.reference_point == "REAR":
                    vehicle_length_add += semantic_model.config.vehicle.ego.wb_rear_axle

                # stop_line_s - vehicle_front < dis_stop_line
                min_position = stop_line_s - dis_stop_line - vehicle_length_add
                # vehicle_front < stop_line_s
                max_position = stop_line_s - vehicle_length_add

                # Adjust the reach node positions
                reach_node.intersect_in_position_domain(
                    p_lon_min=min_position, p_lon_max=max_position
                )

        return [reach_node] if reach_node else []

    def _restrict_reach_node_forbidden(
        self,
        step: int,
        reach_node: ReachNode,
        semantic_model: SemanticModel,
        node_lanelet_ids: Optional[Set[int]] = None,
    ) -> List[ReachNode]:
        # Obtain the id of lanelets with stop lines
        stop_line_lanelet_ids = semantic_model.lanelet_model.stop_line_lanelet_ids

        # Return early if no stop line lanelet IDs are found, full reachable set
        if not stop_line_lanelet_ids:
            return [reach_node]

        # Find lanelet IDs that are both in stop_line_lanelet_ids and node_lanelet_ids
        occupied_lanelet_ids = set(stop_line_lanelet_ids) & set(node_lanelet_ids or [])

        # Return early if no occupied lanelets with stop lines are found
        if not occupied_lanelet_ids:
            return [reach_node]

        result_nodes = [reach_node]  # Initialize with the initial reach_node
        for node in result_nodes:  # Iterate over current result_nodes
            new_nodes = []  # Temporarily store new nodes for this iteration
            for lanelet_id in occupied_lanelet_ids:
                # Obtain the longitudinal coordinate of the stop line
                stop_line_s = self._get_stop_line(semantic_model, lanelet_id)

                if stop_line_s is not None:
                    # Calculate positions relative to the stop line
                    dis_stop_line = semantic_model.config.traffic_rule.dis_stop_line

                    # Additional consideration of the vehicle length
                    vehicle_length_add = semantic_model.config.vehicle.ego.length / 2
                    if semantic_model.config.planning.reference_point == pycrreach.ReferencePoint.REAR:
                        vehicle_length_add += semantic_model.config.vehicle.ego.wb_rear_axle

                    # behind the stop line where: stop_line_s - vehicle_front > dis_stop_line
                    behind_stop_line_position = (
                        stop_line_s - dis_stop_line - vehicle_length_add
                    )
                    # in front of stop line where: stop_line_s < vehicle_front
                    in_front_stop_line_position = stop_line_s - vehicle_length_add

                    # Clone and adjust the reach node positions
                    behind_sl_node = node.clone()
                    behind_sl_node.intersect_in_position_domain(
                        p_lon_max=behind_stop_line_position
                    )

                    in_front_sl_node = node.clone()
                    in_front_sl_node.intersect_in_position_domain(
                        p_lon_min=in_front_stop_line_position
                    )

                    new_nodes.extend([behind_sl_node, in_front_sl_node])
            result_nodes = new_nodes  # Update result_nodes for the next iteration
        # If no adjustment is made, return the original reach node
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

        # Starting and ending longitudinal coordinate of the stop line
        start_s = convert_coords(*stop_line.start)[0]
        end_s = convert_coords(*stop_line.end)[0]

        return min(start_s, end_s) if start_s and end_s else None
