from abc import ABCMeta, abstractmethod
from typing import List

from commonroad.scenario.lanelet import LineMarking
from commonroad.scenario.traffic_sign import TrafficLightState, TrafficLightDirection

from commonroad_reach_semantic_addon.data_structure.proposition import Proposition as P
from commonroad_reach_semantic_addon.data_structure.semantic_model import SemanticModel


class TrafficRule(metaclass=ABCMeta):
    @classmethod
    @abstractmethod
    def concretize(cls, semantic_model: SemanticModel = None) -> List[str]:
        pass


class NoBackwardDrivingRule(TrafficRule):
    @classmethod
    def concretize(cls, semantic_model: SemanticModel = None) -> List[str]:
        pass


class NoOppositeDrivingRule(TrafficRule):
    @classmethod
    def concretize(cls, semantic_model: SemanticModel = None) -> List[str]:
        specification = f"TPL {P.same_driving_direction()}"

        return [specification]


class LineMarkingRule(TrafficRule):
    @classmethod
    def concretize(cls, semantic_model: SemanticModel = None) -> List[str]:
        set_propositions_forbidden = set()
        for lanelet in semantic_model.local_lanelet_network.lanelets:
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


class TrafficLightRule(TrafficRule):
    @classmethod
    def concretize(cls, semantic_model: SemanticModel = None) -> List[str]:
        config = semantic_model.config
        list_specifications = list()

        step_start = semantic_model.step_start
        step_end = semantic_model.step_end
        for step in range(step_start, step_end + 1):
            time_step = step * round(config.planning.dt / config.scenario.dt)
            set_propositions_forbidden = set()

            for lanelet in semantic_model.local_lanelet_network.lanelets:
                list_ids_successors = lanelet.successor
                for id_traffic_light in lanelet.traffic_lights:
                    traffic_light = semantic_model.local_lanelet_network.find_traffic_light_by_id(id_traffic_light)
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


class RightBeforeLeftRule(TrafficRule):
    @classmethod
    def concretize(cls, semantic_model: SemanticModel = None) -> List[str]:
        """Right before left rule R-IN3 in Sebastian's paper.

        More directions should be added to complete the rule.
        """
        list_specifications = list()
        for id_vehicle in semantic_model.set_ids_vehicles_entering_intersection:
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


class PriorityRule(TrafficRule):
    @classmethod
    def concretize(cls, semantic_model: SemanticModel = None) -> List[str]:
        """Priority rule R-IN4 in Sebastian's paper.

        More directions should be added to complete the rule.
        """
        list_specifications = list()
        for id_vehicle in semantic_model.set_ids_vehicles_entering_intersection:
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


class LeftTurningRule(TrafficRule):
    @classmethod
    def concretize(cls, semantic_model: SemanticModel = None) -> List[str]:
        """Left turning rule R-IN5 in Sebastian's paper.

        conditions on outgoings have been replaced by successors of incomings, due to the reason that the former is
        not effective when a vehicle does not reach its outgoing lanelet.

        condition on on_coming_of(x_o, x_ego) has been removed, due to the reason that this will inactivate the rule
        when the reachable sets reach lanelets that are not oncoming of a vehicle.
        """
        list_specifications = list()
        for id_vehicle in semantic_model.set_ids_vehicles_entering_intersection:
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
