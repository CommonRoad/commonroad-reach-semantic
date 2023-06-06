import logging
from collections import defaultdict, Counter
from functools import reduce
from typing import List, Dict, FrozenSet, Set, Tuple, Iterable, Optional

import more_itertools
from commonroad_reach.data_structure.reach.reach_node import ReachNode
from commonroad_reach.data_structure.reach.reach_polygon import ReachPolygon
from commonroad_reach.utility import reach_operation

import commonroad_reach_semantic.data_structure.reach.predicates as predicates
import commonroad_reach_semantic.utility.reach_operation as semantic_reach_operation
from commonroad_reach_semantic.data_structure.config.semantic_configuration import SemanticConfiguration
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.model_checking.finite_automaton import FiniteAutomaton
from commonroad_reach_semantic.data_structure.reach.semantic_reach_set_py import PySemanticReachableSet
from commonroad_reach_semantic.data_structure.rule.traffic_rule_interface import TrafficRuleInterface

logger = logging.getLogger(__name__)


class PySemanticSplittingOTFReachableSet(PySemanticReachableSet):
    """
    Reachable set computation considering temporal constraints on-the-fly with Python backend.
    """

    config: SemanticConfiguration
    dict_step_to_states_to_drivable_area: Dict[int, Dict[FrozenSet[int], List[ReachPolygon]]]
    dict_step_to_states_to_propagated_set: Dict[int, Dict[FrozenSet[int], List[ReachNode]]]
    reachable_set_to_label: Dict[ReachNode, FrozenSet[int]]
    automaton: FiniteAutomaton

    def __init__(self, config: SemanticConfiguration, semantic_model: SemanticModel,
                 rule_interface: TrafficRuleInterface):
        super().__init__(config, semantic_model, rule_interface)

        self.dict_step_to_states_to_drivable_area = dict()
        self.dict_step_to_states_to_propagated_set = dict()

        self.reachable_set_to_label = dict()

        self._initialize_zero_state_polygons()

        # Construct finite automaton from traffic rules
        concatenated_specifications = self.rule_interface.get_combined_ltl_specs()
        self.automaton = FiniteAutomaton(concatenated_specifications)

        # Compute initial reachable set
        initial_reachable_sets = self._construct_initial_reachable_sets()

        # Label initial state with propositions and automaton states
        initial_reachable_sets = more_itertools.flatten(
            self._split_reachable_set(self.step_start, initial_reachable_set)
            for initial_reachable_set in initial_reachable_sets
        )
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
            self.dict_step_to_states_to_drivable_area[step] = dict()
            self.dict_step_to_states_to_propagated_set[step] = dict()
            self.dict_step_to_propagated_set[step] = list()
            return None

        propagated_sets = self._propagate_reachable_set(reachable_set_previous)

        propagated_sets = more_itertools.flatten(
            self._split_reachable_set(step, propagated_set)
            for propagated_set in propagated_sets
        )
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

        self.dict_step_to_drivable_area[step] = list(more_itertools.flatten(dict_states_to_drivable_area.values()))
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

    def _filter_reachable_sets(self, reachable_sets: Iterable[ReachNode], step: int) -> List[ReachNode]:
        """Filter reachable sets that cannot be part of an accepting run of the automaton."""
        is_final_step = (step == self.step_end)
        return [
            reachable_set for reachable_set in reachable_sets
            if self.reachable_set_to_label[reachable_set] and (
                    not is_final_step or self._has_accepting_state(reachable_set))
        ]

    def _has_accepting_state(self, reachable_set: ReachNode) -> bool:
        """Check if the given reachable set has an accepting state."""
        return any(self.automaton.is_accepting_state(state) for state in self.reachable_set_to_label[reachable_set])

    def _split_reachable_set(self, step: int, reachable_set: ReachNode) -> List[ReachNode]:
        """Split the given reachable set along the transitions of the automaton states of its propagation source.

        For this, consider the outgoing transitions of all automaton states in the labels of the propagation source.
        We then split and cut the reachable set along the conditions of these transitions.
        :param step: Current step of the reachability analysis.
        :param reachable_set: The reachable set to split.
        :return: List of reachable sets so that each is a subset of the given reachable set, and satisfies the condition of at least one transition (up to overapproximation).
        """
        current_states = frozenset({self.automaton.initial_state}) if step == self.step_start else \
            self.reachable_set_to_label[reachable_set.source_propagation]
        transitions = self.automaton.non_deterministic_transitions_from(current_states)
        constrained_reachable_sets = self._split_to_minterms(step, [reachable_set], transitions)

        return constrained_reachable_sets

    def _split_to_minterms(self, step: int, reachable_sets: List[ReachNode],
                           transitions: Dict[Tuple[Tuple[str, bool]], Set[int]],
                           finished_literals: List[Tuple[str, bool]] = None, regionized: bool = False) -> List[
        ReachNode]:
        """Split the given reachable sets along the given transitions.

        This is a recursive function that splits the reachable sets along the given transitions.
        The goal is to reuse as many splits as possible.
        First, we select a (not yet finished) literal that we will use for splitting in this step.
        Then, we partition the transitions into those that depend on the literal and those that do not.
        To further handle the former, we need to restrict the reachable sets to the chosen literal.
        The chosen literal is marked as finished for the restricted reachable sets.
        Thus, when _split_to_minterms is called, all nodes in reachable_sets satisfy all literals in finished_literals.
        We then recursively split the original reachable sets along the transitions that do not depend on the literal,
        and the restricted reachable sets along the transitions that do depend on the literal.
        The recursion ends, when there are no more reachable sets, because restricting them along the literal resulted in an empty set.
        The recursion also ends, when we considered all literals.
        In this case, we label the reachable sets with the target states of the transitions that they satisfy.

        :param step: Current step of the reachability analysis.
        :param reachable_sets: Reachable sets to split.
        :param transitions: Transitions to split along.
        :param finished_literals: Literals that we no longer have to consider.
        :param regionized: Whether the reachable sets are already split into regions.
        :return: List of reachable sets so that each is a subset of the given reachable sets, and satisfies the condition of at least one transition (up to overapproximation).
        """

        if not reachable_sets or not transitions:
            # if there are no reachable sets or no transitions, there is nothing to split
            return reachable_sets

        # select the next literal to split on
        finished_literals = finished_literals or []
        literal_to_split = self._choose_next_literal(transitions.keys(), finished_literals)

        # BASE CASE: if there is no literal to split, we are done
        if not literal_to_split:
            # there should always be exactly one transition left at this point
            assert len(transitions) == 1
            # all nodes in reachable_sets satisfy the transition condition, so label them with the destination states
            for reachable_set in reachable_sets:
                self.reachable_set_to_label[reachable_set] = frozenset(reduce(set.union, transitions.values()))
            return reachable_sets

        # partition the transitions into those whose label needs the literal and those that don't
        not_needs_literal, needs_literal = self._partition_transitions(literal_to_split, transitions)

        # if there are transitions that don't need the current literal we have to clone the reach nodes before restricting
        # so that we can keep the original nodes for those transitions
        restricted_reachable_sets, restriction_regionized = self._restrict_to_literal(step, reachable_sets,
                                                                                      literal_to_split, regionized,
                                                                                      clone=bool(not_needs_literal))

        # recurse to split along the remaining literals
        if not_needs_literal:
            # note that only the restricted reachable sets might have been regionized
            return self._split_to_minterms(step, reachable_sets, not_needs_literal, finished_literals, regionized) \
                + self._split_to_minterms(step, restricted_reachable_sets, needs_literal,
                                          finished_literals + [literal_to_split], regionized or restriction_regionized)
        else:
            return self._split_to_minterms(step, restricted_reachable_sets, needs_literal,
                                           finished_literals + [literal_to_split], regionized or restriction_regionized)

    def _restrict_to_literal(self, step: int, reachable_sets: List[ReachNode], literal: Tuple[str, bool],
                             regionized: bool, clone: bool = True) -> Tuple[List[ReachNode], bool]:
        """Restrict the reachable sets to the given literal.

        If restricting requires lanelet information, we first split the reachable sets into regions (if we haven't already).
        If we clone the reachable sets, we also copy the labels from the original nodes to the clones.
        :param step: Current step of the reachability analysis.
        :param reachable_sets: The reachable sets to restrict.
        :param literal: The literal to restrict to.
        :param regionized: Whether the reachable sets are already split into regions.
        :param clone: Whether to clone the reachable sets before restricting.
        :return: The restricted reachable sets and whether they were split into regions.
        """
        to_restrict = [node.clone() for node in reachable_sets] if clone else reachable_sets

        if clone and regionized:
            # if we already split to regions, we need to copy labels from the original nodes to the clones
            for src, dst in zip(reachable_sets, to_restrict):
                self.labeler.copy_labels(src, dst)
        pred = predicates.from_proposition(*literal)
        if pred.needs_lanelets and not regionized:
            # if the predicate needs lanelets, we need to split the reachable sets into regions first (if we haven't already)
            to_restrict = list(more_itertools.flatten(
                self.labeler.split_wrt_regions(step, restricted_reachable_set)
                for restricted_reachable_set in to_restrict
            ))
        # restrict the reachable sets to the predicate
        restricted_reachable_sets = list(more_itertools.flatten(
            pred.restrict_reach_node(step, node, self.labeler.semantic_model,
                                     node_lanelet_ids=self.labeler.reachable_set_to_lanelet_ids[
                                         node] if pred.needs_lanelets else None)
            for node in to_restrict
        ))
        return restricted_reachable_sets, pred.needs_lanelets

    @staticmethod
    def _partition_transitions(literal: Tuple[str, bool], transitions: Dict[Tuple[Tuple[str, bool]], Set[int]]) \
            -> Tuple[Dict[Tuple[Tuple[str, bool]], Set[int]], Dict[Tuple[Tuple[str, bool]], Set[int]]]:
        """Partition the transitions into those that depend on the literal and those that don't.

        :param literal: The literal to partition the transitions along.
        :param transitions: The transitions to partition.
        :return: A tuple of two dictionaries, the first containing the transitions that don't depend on the literal, the second containing the transitions that do.
        """
        not_needs_literal, needs_literal = more_itertools.partition(lambda trans: literal in trans[0],
                                                                    transitions.items())
        return dict(not_needs_literal), dict(needs_literal)

    @staticmethod
    def _choose_next_literal(minterms: Iterable[Tuple[Tuple[str, bool]]], ignored_literals: List[Tuple[str, bool]]) -> \
            Optional[Tuple[str, bool]]:
        """Selects the next literal along which to split the reachable set.

        We use a greedy approach, so we choose the literal that occurs most often in the minterms.
        :param minterms: The list of minterms to consider.
        :param ignored_literals: These literals will be ignored when choosing the next literal.
        :return: The literal that occurs most often in minterms.
        """
        # we can simply flatten the list here, since no minterm contains the same literal twice
        literals = [literal for literal in more_itertools.flatten(minterms) if literal not in ignored_literals]
        if not literals:
            return None
        c = Counter(literals)

        # the literals that occur most often are candidates for the next literal
        max_cnt = max(c.values())
        candidates = [literal for literal, cnt in c.items() if cnt == max_cnt]

        # prefer predicates that don't need lanelets, as this avoids splitting to regions
        # TODO: we could choose a different ordering here or make this configurable
        candidates = sorted(candidates, key=lambda literal: predicates.from_proposition(*literal).needs_lanelets)

        return next(candidates.__iter__(), None)
