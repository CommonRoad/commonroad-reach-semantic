from __future__ import annotations

import enum
from enum import Enum, auto
from typing import List, Set, Optional

from commonroad.scenario.lanelet import LineMarking, LaneletType, Lanelet, LaneletNetwork
from commonroad.scenario.traffic_light import TrafficLightState, TrafficLightDirection

from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.environment_model.vehicle import Vehicle
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
    ENTERING_VEHICLES = auto()

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
            case "EnteringVehiclesRule":
                return TrafficRule.ENTERING_VEHICLES
            case _:
                raise ValueError(f"Rule {rule_string} is not supported.")

    def concretize(self, semantic_model: SemanticModel = None) -> List[str]:
        match self:
            case TrafficRule.NO_BACKWARD_DRIVING:
                return [f"LTL G !{P.drives_backward()}"]
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
            case TrafficRule.ENTERING_VEHICLES:
                return _concretize_entering_vehicles_rule(semantic_model)


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


def _concretize_entering_vehicles_rule(semantic_model: SemanticModel) -> List[str]:
    """Entering vehicles rule R-I5 in Sebastian's paper.

    G (
        on_main_carriageway(ego) & in_front_of(ego, other) & on_access_ramp(other) & F on_main_carriageway(other) ->
        !(!main_carriagway_right_lane(ego) & F main_carriagway_right_lane(ego))
    )
    joined by conjunction for all other vehicles

    Note that the rule requires that the other vehicle is in front of the ego vehicle.
    We model this by requiring that the ego vehicle is behind the other vehicle, because our predicates are defined
    from the ego vehicle's perspective.
    """
    list_specifications = list()
    step_start = semantic_model.config.planning.step_start
    step_end = step_start + semantic_model.config.planning.steps_computation

    access_ramp_lanelet_ids = semantic_model.lanelet_model.access_ramp_lanelet_ids
    main_carriageway_lanelet_ids = semantic_model.lanelet_model.main_carriageway_lanelet_ids
    right_lane_lanelet_ids = semantic_model.lanelet_model.right_lane_lanelet_ids

    if not access_ramp_lanelet_ids or not main_carriageway_lanelet_ids or not right_lane_lanelet_ids:
        # there cannot be an entering vehicle if there is no access ramp or main carriageway
        # if there is no rightmost lane, the consequence of the implication would always be true
        return list_specifications

    for vehicle in semantic_model.vehicle_model.list_vehicles:
        if not _is_entering_vehicle(vehicle, access_ramp_lanelet_ids, main_carriageway_lanelet_ids, step_start, step_end):
            # only entering vehicles are relevant for this rule
            continue

        specification_ltl = \
            f"G (" + \
                f"{P.on_main_carriageway()} & {P.behind(vehicle.id_vehicle)} & {P.vehicle_on_access_ramp(vehicle.id_vehicle)} & F {P.vehicle_on_main_carriageway(vehicle.id_vehicle)} ->" + \
                f"!(!{P.on_right_lane()} & F {P.on_right_lane()})" + \
            f")"
        list_specifications.append(f"LTL {specification_ltl}\n")

    return list_specifications


def _is_entering_vehicle(vehicle: Vehicle, access_ramp_lanelet_ids: Set[int], main_carriageway_lanelet_ids: Set[int], step_start: int, step_end: int) -> bool:
    """Determine if vehicle is entering the main carriageway.

    Evaluates F (on_access_ramp(vehicle) & F on_main_carriageway(vehicle))
    If this is true, the vehicle is an entering vehicle.
    """
    access_ramp = False
    carriageway_after_ramp = False
    for step in range(step_start, step_end + 1):
        lanelet_ids = set(vehicle.lanelet_ids_at_step(step))
        access_ramp = access_ramp or not lanelet_ids.isdisjoint(access_ramp_lanelet_ids)
        if access_ramp:
            # only check for main carriageway if access ramp has been found
            # vehicle needs to be on ramp first and then on main carriageway
            carriageway_after_ramp = carriageway_after_ramp or not lanelet_ids.isdisjoint(main_carriageway_lanelet_ids)
        if access_ramp and carriageway_after_ramp:
            break
    return access_ramp and carriageway_after_ramp
