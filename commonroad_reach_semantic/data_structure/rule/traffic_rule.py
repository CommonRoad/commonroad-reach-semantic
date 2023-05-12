from __future__ import annotations

import enum
from enum import Enum, auto
from typing import List

from commonroad.scenario.lanelet import LineMarking
from commonroad.scenario.traffic_sign import TrafficLightState, TrafficLightDirection

from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.rule.proposition import Proposition as P


@enum.unique
class TrafficRule(Enum):
    NO_BACKWARD_DRIVING = auto()
    NO_OPPOSITE_DRIVING = auto()
    LINE_MARKING = auto()
    TRAFFIC_LIGHT = auto()
    RIGHT_BEFORE_LEFT = auto()
    PRIORITY = auto()
    LEFT_TURNING = auto()

    @staticmethod
    def from_string(rule_string: str) -> TrafficRule:
        match rule_string:
            case "NoBackwardDrivingRule":
                return TrafficRule.NO_BACKWARD_DRIVING
            case "NoOppositeDrivingRule":
                return TrafficRule.NO_OPPOSITE_DRIVING
            case "LineMarkingRule":
                return TrafficRule.LINE_MARKING
            case "TrafficLightRule":
                return TrafficRule.TRAFFIC_LIGHT
            case "RightBeforeLeftRule":
                return TrafficRule.RIGHT_BEFORE_LEFT
            case "PriorityRule":
                return TrafficRule.PRIORITY
            case "LeftTurningRule":
                return TrafficRule.LEFT_TURNING
            case _:
                raise ValueError(f"Rule {rule_string} is not supported.")

    def concretize(self, semantic_model: SemanticModel = None) -> List[str]:
        match self:
            case TrafficRule.NO_BACKWARD_DRIVING:
                return []
            case TrafficRule.NO_OPPOSITE_DRIVING:
                return [f"TPL {P.same_driving_direction()}"]
            case TrafficRule.LINE_MARKING:
                return _concretize_line_marking_rule(semantic_model)
            case TrafficRule.TRAFFIC_LIGHT:
                return _concretize_traffic_light_rule(semantic_model)
            case TrafficRule.RIGHT_BEFORE_LEFT:
                return _concretize_right_before_left_rule(semantic_model)
            case TrafficRule.PRIORITY:
                return _concretize_priority_rule(semantic_model)
            case TrafficRule.LEFT_TURNING:
                return _concretize_left_turning_rule(semantic_model)


def _concretize_line_marking_rule(semantic_model: SemanticModel) -> List[str]:
    set_propositions_forbidden = set()
    for lanelet in semantic_model.lanelet_model.local_lanelet_network.lanelets:
        if lanelet.adj_left and lanelet.line_marking_left_vertices in [LineMarking.SOLID,
                                                                       LineMarking.BROAD_SOLID]:
            set_propositions_forbidden.add(f"!{P.lanelet_transition(lanelet.lanelet_id, lanelet.adj_left)}")

        if lanelet.adj_right and lanelet.line_marking_right_vertices in [LineMarking.SOLID,
                                                                         LineMarking.BROAD_SOLID]:
            set_propositions_forbidden.add(f"!{P.lanelet_transition(lanelet.lanelet_id, lanelet.adj_right)}")

    if set_propositions_forbidden:
        return [f"TPL {' & '.join(set_propositions_forbidden)}"]

    else:
        return []


def _concretize_traffic_light_rule(semantic_model: SemanticModel) -> List[str]:
    config = semantic_model.config
    list_specifications = list()

    step_start = semantic_model.step_start
    step_end = semantic_model.step_end
    for step in range(step_start, step_end + 1):
        time_step = step * round(config.planning.dt / config.scenario.dt)
        set_propositions_forbidden = set()

        for lanelet in semantic_model.lanelet_model.local_lanelet_network.lanelets:
            list_ids_successors = lanelet.successor
            for id_traffic_light in lanelet.traffic_lights:
                traffic_light = semantic_model.lanelet_model.local_lanelet_network.find_traffic_light_by_id(id_traffic_light)
                state_light = traffic_light.get_state_at_time_step(time_step)
                direction_light = traffic_light.direction

                if state_light not in [TrafficLightState.RED, TrafficLightState.RED_YELLOW]:
                    continue

                if direction_light == TrafficLightDirection.ALL:
                    for id_successor in list_ids_successors:
                        set_propositions_forbidden.add(f"!{P.lanelet_transition(lanelet.lanelet_id, id_successor)}")

                else:
                    # todo: complete this of partial directions
                    pass

        if set_propositions_forbidden:
            list_specifications.append(f"TPL [{step}] {' & '.join(set_propositions_forbidden)}")

    return list_specifications


def _concretize_right_before_left_rule(semantic_model: SemanticModel) -> List[str]:
    """Right before left rule R-IN3 in Sebastian's paper.

    More directions should be added to complete the rule.
    """
    list_specifications = list()
    for id_vehicle in semantic_model.vehicle_model.set_ids_vehicles_entering_intersection:
        o = id_vehicle
        specification_mtl = \
            f"G (" \
                f"(" \
                    f"({P.intersection_left_of(o)} & !{P.has_relevant_traffic_light()}) & " \
                    f"(F ({P.in_straight_successor()}) & F ({P.in_straight_successor(o)}) & {P.same_straight_straight_priority(o)})" \
                f")" \
                f" -> " \
                f"(!{P.in_intersection()} | {P.not_endanger_mtl(o)})" \
            f")"

        specification_ctl = \
            f"(" \
                f"({P.intersection_left_of(o)} & !{P.has_relevant_traffic_light()}) & " \
                f"(AF ({P.in_straight_successor()}) & EF ({P.in_straight_successor(o)}) & {P.same_straight_straight_priority(o)})" \
                f")" \
            f" -> " \
            f"(!{P.in_intersection()} | {P.not_endanger_ctl(o)})"

        list_specifications.append(f"MTL {specification_mtl}\n")
        list_specifications.append(f"CTL {specification_ctl}\n")

    return list_specifications


def _concretize_priority_rule(semantic_model: SemanticModel) -> List[str]:
    """Priority rule R-IN4 in Sebastian's paper.

    More directions should be added to complete the rule.
    """
    list_specifications = list()
    for id_vehicle in semantic_model.vehicle_model.set_ids_vehicles_entering_intersection:
        o = id_vehicle
        specification_mtl = \
            f"G (" \
                f"(" \
                    f"F ({P.in_straight_successor()}) & F ({P.in_straight_successor(o)}) & " \
                    f"({P.no_straight_straight_priority(o)} | {P.same_straight_straight_priority(o)})" \
                f")" \
                f" -> " \
                f"(!{P.in_intersection()} | {P.not_endanger_mtl(o)})" \
            f")"

        specification_ctl = \
            f"(" \
                f"AF ({P.in_straight_successor()}) & EF ({P.in_straight_successor(o)}) & " \
                f"({P.no_straight_straight_priority(o)} | {P.same_straight_straight_priority(o)})" \
            f")" \
            f" -> " \
            f"(!{P.in_intersection()} | {P.not_endanger_ctl(o)})"

        list_specifications.append(f"MTL {specification_mtl}\n")
        list_specifications.append(f"CTL {specification_ctl}\n")

    return list_specifications


def _concretize_left_turning_rule(semantic_model: SemanticModel) -> List[str]:
    """Left turning rule R-IN5 in Sebastian's paper.

    conditions on outgoings have been replaced by successors of incomings, due to the reason that the former is
    not effective when a vehicle does not reach its outgoing lanelet.

    condition on on_coming_of(x_o, x_ego) has been removed, due to the reason that this will inactivate the rule
    when the reachable sets reach lanelets that are not oncoming of a vehicle.
    """
    list_specifications = list()
    for id_vehicle in semantic_model.vehicle_model.set_ids_vehicles_entering_intersection:
        o = id_vehicle
        specification_mtl = \
            f"G (" \
                f"(" \
                    f"F ({P.in_left_successor()}) & " \
                    f"(" \
                        f"({P.no_left_straight_priority(o)} | {P.same_left_straight_priority(o)}) & " \
                        f"F ({P.in_straight_successor(o)})" \
                    f")" \
                f")" \
                f" -> " \
                f"{P.not_endanger_mtl(o)}" \
            f")"

        specification_ctl = \
            f"(" \
                f"AF ({P.in_left_successor()}) & " \
                f"(" \
                    f"({P.no_left_straight_priority(o)} | {P.same_left_straight_priority(o)}) & " \
                    f"EF ({P.in_straight_successor(o)})" \
                f")" \
            f")" \
            f" -> " \
            f"{P.not_endanger_ctl(o)}"

        list_specifications.append(f"MTL {specification_mtl}\n")
        list_specifications.append(f"CTL {specification_ctl}\n")

    return list_specifications
