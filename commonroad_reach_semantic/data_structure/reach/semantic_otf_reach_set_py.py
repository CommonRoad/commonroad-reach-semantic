import more_itertools
import logging
from collections import defaultdict
from typing import List, Dict, FrozenSet

from commonroad_reach.data_structure.reach.reach_node import ReachNode
from commonroad_reach.data_structure.reach.reach_polygon import ReachPolygon
from commonroad_reach.utility import reach_operation

import commonroad_reach_semantic.utility.reach_operation as semantic_reach_operation
from commonroad_reach_semantic.data_structure.config.semantic_configuration import SemanticConfiguration
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.model_checking.finite_automaton import FiniteAutomaton
from commonroad_reach_semantic.data_structure.reach.semantic_reach_set_py import PySemanticReachableSet
from commonroad_reach_semantic.data_structure.rule.traffic_rule_interface import TrafficRuleInterface

logger = logging.getLogger(__name__)


class PySemanticOTFReachableSet(PySemanticReachableSet):
    """
    Reachable set computation considering temporal constraints on-the-fly with Python backend.
    """

    config: SemanticConfiguration
    dict_step_to_states_to_drivable_area: Dict[int, Dict[FrozenSet[int], List[ReachPolygon]]]
    dict_step_to_states_to_propagated_set: Dict[int, Dict[FrozenSet[int], List[ReachNode]]]
    reachable_set_to_label: Dict[ReachNode, FrozenSet[int]]

    def __init__(self, config: SemanticConfiguration, semantic_model: SemanticModel,
                 rule_interface: TrafficRuleInterface):
        super().__init__(config, semantic_model, rule_interface)

        self.dict_step_to_states_to_drivable_area = dict()
        self.dict_step_to_states_to_propagated_set = dict()

        self.reachable_set_to_label = dict()

        self._initialize_zero_state_polygons()

        # Construct finite automaton from traffic rules
        concatenated_specifications = "(" + ") & (".join(self.rule_interface.list_specifications_ltl) + ")"
        self.automaton = FiniteAutomaton(concatenated_specifications)

        # Compute initial reachable set
        initial_reachable_sets = self._construct_initial_reachable_sets()

        # Label initial state with propositions and automaton states
        self.labeler.label_initial_state(initial_reachable_sets, self.step_start)
        self._label_reachable_sets_with_automaton_states(initial_reachable_sets, initial_step=True)
        initial_reachable_sets = self._filter_reachable_sets(initial_reachable_sets, self.step_start)

        # Compute initial drivable area
        self.dict_step_to_reachable_set[self.step_start] = initial_reachable_sets
        self.dict_step_to_drivable_area[self.step_start] = reach_operation.project_propagated_sets_to_position_domain(
            self.dict_step_to_reachable_set[self.step_start])

        logger.debug("PySemanticOTFReachableSet initialized.")

    def _compute_drivable_area_at_step(self, step: int):
        """
        Computes drivable area for the given step.

        Steps:
            1. Propagate each node of the reachable set from the previous step, resulting in propagated base sets.
            2. Split and label propagated sets according to relevant atomic propositions.
            2. Project base sets onto the position domain to obtain position rectangles.
            3. Merge, repartition and check collisions for these rectangles. The order depends on the configuration.
        """
        reachable_set_previous = self.dict_step_to_reachable_set[step - 1]

        if len(reachable_set_previous) < 1:
            self.dict_step_to_drivable_area[step] = list()
            self.dict_step_to_propagated_set[step] = list()
            return None

        propagated_sets = self._propagate_reachable_set(reachable_set_previous)

        # split w.r.t regions and position intervals
        propagated_sets = more_itertools.flatten(
            self.labeler.split_wrt_regions(step, propagated_set)
            for propagated_set in propagated_sets
        )
        propagated_sets = more_itertools.flatten(
            self.labeler.split_wrt_position_intervals(step, propagated_set)
            for propagated_set in propagated_sets
        )

        # discard the ones colliding with vehicles
        propagated_sets = self.labeler.discard_colliding_nodes(propagated_sets)

        # examine whether the propagated sets satisfy TPL specifications
        propagated_sets = self.rule_interface.tpl_checker.examine_tpl_specifications(step, propagated_sets,
                                                                                     self.labeler.reachable_set_to_propositions)

        # update traffic propositions of the propagated sets
        propagated_sets = self.labeler.label_traffic_propositions(step, propagated_sets)

        # label propagated sets with automaton states and filter
        self._label_reachable_sets_with_automaton_states(propagated_sets)
        propagated_sets = self._filter_reachable_sets(propagated_sets, step)

        # partition propagated sets by their automaton states
        dict_states_to_propagated_set: Dict[FrozenSet[int], List[ReachNode]] = defaultdict(list)
        for propagated_set in propagated_sets:
            dict_states_to_propagated_set[self.reachable_set_to_label[propagated_set]].append(propagated_set)

        # merge, collision check, and repartition propagated sets partitioned by their automaton states,
        # because we must not merge sets with different states
        dict_states_to_drivable_area = dict()
        for automaton_states, propagated_sets_per_proposition in dict_states_to_propagated_set.items():
            list_rectangles_projected = reach_operation.project_propagated_sets_to_position_domain(
                propagated_sets_per_proposition)
            dict_states_to_drivable_area[automaton_states] = self._collision_check_and_repartition(
                list_rectangles_projected, step)

        self.dict_step_to_drivable_area[step] = list(
            more_itertools.flatten(dict_states_to_drivable_area.values()))
        self.dict_step_to_states_to_drivable_area[step] = dict_states_to_drivable_area
        self.dict_step_to_states_to_propagated_set[step] = dict_states_to_propagated_set
        self.dict_step_to_propagated_set[step] = propagated_sets

    def _compute_reachable_set_at_step(self, step):
        """
        Computes reachable set for the given step.

        Steps:
            1. construct reach nodes from drivable area and the propagated sets.
            2. update parent-child relationship of the nodes.
        """
        dict_states_to_propagated_set = self.dict_step_to_states_to_propagated_set[step]
        dict_states_to_drivable_area = self.dict_step_to_states_to_drivable_area[step]

        if not dict_states_to_drivable_area:
            self.dict_step_to_reachable_set[step] = list()
            return None

        # discard drivable area with small area if there are more than one node (this is subject to change)
        num_drivable_area = sum(
            [len(list_drivable) for list_drivable in dict_states_to_drivable_area.values()])
        discard_small_node = (num_drivable_area > 1)

        # work with the reachable sets partitioned by automaton states here, because otherwise it could happen
        # that we merge two reachable sets with different states when they intersect with the same drivable area
        dict_propositions_to_reachable_set = dict()
        for automaton_states, drivable_area in dict_states_to_drivable_area.items():
            propagated_sets = dict_states_to_propagated_set[automaton_states]

            list_nodes = reach_operation.construct_reach_nodes(drivable_area, propagated_sets)
            if discard_small_node:
                list_nodes = semantic_reach_operation.discard_nodes_with_short_edge(list_nodes,
                                                                                    self.config.reachable_set.length_edge_node_min)
            if list_nodes:
                reachable_sets = reach_operation.connect_children_to_parents(step, list_nodes)
                # assign label to all newly constructed reach nodes
                for node in reachable_sets:
                    self.reachable_set_to_label[node] = automaton_states
                dict_propositions_to_reachable_set[automaton_states] = reachable_sets

        self.dict_step_to_reachable_set[step] = list(
            more_itertools.flatten(dict_propositions_to_reachable_set.values()))

    def _label_reachable_sets_with_automaton_states(self, reachable_sets: List[ReachNode],
                                                    initial_step: bool = False) -> None:
        for reachable_set in reachable_sets:
            automaton_states = self.reachable_set_to_label[
                reachable_set.source_propagation] if not initial_step else {self.automaton.initial_state}
            for automaton_state in automaton_states:
                self._label_automaton_states(reachable_set, automaton_state)

    def _label_automaton_states(self, reachable_set: ReachNode, current_state: int) -> None:
        """Label the reachable set with the automaton states that are reachable given its propositions."""
        reach_props = self.labeler.reachable_set_to_propositions[reachable_set].set_propositions
        automaton_states = set()
        for next_state, minterms in self.automaton.transitions_from(current_state):
            for minterm in minterms:
                positive_props = [proposition for proposition, negated in minterm if not negated]
                negative_props = [proposition for proposition, negated in minterm if negated]
                if reach_props.issuperset(positive_props) and reach_props.isdisjoint(negative_props):
                    automaton_states.add(next_state)
                    break  # inner loop
        self.reachable_set_to_label[reachable_set] = frozenset(automaton_states)

    def _filter_reachable_sets(self, reachable_sets: List[ReachNode], step: int) -> List[ReachNode]:
        """Filter reachable sets that cannot be part of an accepting run of the automaton."""
        is_final_step = (step == self.step_end)
        return [
            reachable_set for reachable_set in reachable_sets
            if self.reachable_set_to_label[reachable_set] and (
                    not is_final_step or self._has_accepting_state(reachable_set))
        ]

    def _has_accepting_state(self, reachable_set: ReachNode) -> bool:
        return any(self.automaton.is_accepting_state(state) for state in self.reachable_set_to_label[reachable_set])

    def _split_reachable_set(self, reachable_set: ReachNode, current_state: int) -> List[tuple[ReachNode, int]]:
        split_sets = list()
        for next_state, minterms in self.automaton.transitions_from(current_state):
            for minterm in minterms:
                constrained_reachable_set = reachable_set.clone()
                for proposition, negated in minterm:
                    # consider only position propositions for now
                    # TODO: Constrain reachable set to (negated) proposition
                    if negated:
                        pass
                    else:
                        # see split_wrt_regions, so something like
                        # constrained_reachable_set.intersect_in_position_domain(*proposition.area.intersection(reachable_set.position_rectangle).bounds)
                        pass
                    pass
                # TODO: Check if constrained reachable set is empty --> only add if not empty
                split_sets.append((constrained_reachable_set, next_state))
        return split_sets
