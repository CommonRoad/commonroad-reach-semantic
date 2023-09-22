import logging
from collections import defaultdict
from typing import List, Dict, FrozenSet, Tuple, Iterable

import more_itertools
from commonroad_reach.data_structure.reach.reach_node import ReachNode
from commonroad_reach.data_structure.reach.reach_polygon import ReachPolygon
from commonroad_reach.utility import reach_operation

import commonroad_reach_semantic.utility.reach_operation as semantic_reach_operation
from commonroad_reach_semantic.data_structure.config.semantic_configuration import SemanticConfiguration
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.model_checking.finite_automaton import FiniteAutomaton, State
from commonroad_reach_semantic.data_structure.reach.semantic_reach_set_py import PySemanticReachableSet
from commonroad_reach_semantic.data_structure.rule.traffic_rule_interface import TrafficRuleInterface

logger = logging.getLogger(__name__)


class PySemanticOTFReachableSet(PySemanticReachableSet):
    """
    Reachable set computation considering temporal constraints on-the-fly with Python backend.
    """

    config: SemanticConfiguration
    step_to_states_to_drivable_area: Dict[int, Dict[Tuple[FrozenSet[State], FrozenSet[State]], List[ReachPolygon]]]
    step_to_states_to_propagated_set: Dict[int, Dict[Tuple[FrozenSet[State], FrozenSet[State]], List[ReachNode]]]
    reachable_set_to_label: Dict[ReachNode, FrozenSet[State]]
    automaton: FiniteAutomaton

    def __init__(self, config: SemanticConfiguration, semantic_model: SemanticModel,
                 rule_interface: TrafficRuleInterface):
        super().__init__(config, semantic_model, rule_interface)

        self.step_to_states_to_drivable_area = dict()
        self.step_to_states_to_propagated_set = dict()

        self.reachable_set_to_label = dict()

        self._initialize_zero_state_polygons()

        # Construct finite automaton from traffic rules
        self.automaton = FiniteAutomaton(self.rule_interface.list_specifications_ltl, config.traffic_rule.mode_automata)

        # Compute initial reachable set
        self.compute_drivable_area_at_step(self.step_start)
        self.compute_reachable_set_at_step(self.step_start)

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
        if step != self.step_start:
            reachable_set_previous = self.dict_step_to_reachable_set[step - 1]

            if len(reachable_set_previous) < 1:
                self.dict_step_to_drivable_area[step] = list()
                self.step_to_states_to_drivable_area[step] = dict()
                self.step_to_states_to_propagated_set[step] = dict()
                self.dict_step_to_propagated_set[step] = list()
                return None

            propagated_sets = self._propagate_reachable_set(reachable_set_previous)
        else:
            # there is no preceding reachable set to propagate in the initial step
            # we also don't need one as we have to use the set of initial states anyway
            propagated_sets = self._construct_initial_reachable_sets()

        propagated_sets = list(more_itertools.flatten(
            self._split_reachable_set(step, propagated_set)
            for propagated_set in propagated_sets
        ))

        # partition propagated sets by their automaton states and the states of their propagation source
        dict_states_to_propagated_set: Dict[Tuple[FrozenSet[int], FrozenSet[int]], List[ReachNode]] = defaultdict(list)
        for propagated_set in propagated_sets:
            if step != self.step_start:
                key = (self.reachable_set_to_label[propagated_set.source_propagation],
                       self.reachable_set_to_label[propagated_set])
            else:
                key = (frozenset({self.automaton.initial_state}), self.reachable_set_to_label[propagated_set])
            dict_states_to_propagated_set[key].append(propagated_set)

        # merge, collision check, and repartition propagated sets
        # this is done individually for each group calculated above, because we must not merge sets semantically different base sets
        # it is necessary to also consider the states of the propagation source for the partitioning, because only if these are equal, the automaton cannot distinguish the base sets
        # if only the target states were considered, the automaton could possibly distinguish them if the source states reach the target state via different propositions
        dict_states_to_drivable_area = dict()
        for automaton_states, propagated_sets_per_proposition in dict_states_to_propagated_set.items():
            list_rectangles_projected = reach_operation.project_propagated_sets_to_position_domain(
                propagated_sets_per_proposition)
            dict_states_to_drivable_area[automaton_states] = self._collision_check_and_repartition(
                list_rectangles_projected, step)

        self.dict_step_to_drivable_area[step] = list(more_itertools.flatten(dict_states_to_drivable_area.values()))
        self.step_to_states_to_drivable_area[step] = dict_states_to_drivable_area
        self.step_to_states_to_propagated_set[step] = dict_states_to_propagated_set
        self.dict_step_to_propagated_set[step] = propagated_sets

    def _compute_reachable_set_at_step(self, step: int):
        """
        Computes reachable set for the given step.

        Steps:
            1. construct reach nodes from drivable area and the propagated sets.
            2. update parent-child relationship of the nodes.
        """
        states_to_propagated_set = self.step_to_states_to_propagated_set[step]
        states_to_drivable_area = self.step_to_states_to_drivable_area[step]

        if not states_to_drivable_area:
            self.dict_step_to_reachable_set[step] = list()
            return None

        # discard drivable area with small area if there are more than one node (this is subject to change)
        num_drivable_area = sum(
            [len(list_drivable) for list_drivable in states_to_drivable_area.values()])
        discard_small_node = (num_drivable_area > 1) and self.config.reachable_set.discard_small_nodes

        # work with the reachable sets partitioned by automaton states here, because otherwise it could happen
        # that we merge two reachable sets with different states when they intersect with the same drivable area
        new_reachable_sets = list()
        for automaton_states, drivable_area in states_to_drivable_area.items():
            propagated_sets = states_to_propagated_set[automaton_states]

            reachable_sets = reach_operation.construct_reach_nodes(drivable_area, propagated_sets)
            if discard_small_node:
                reachable_sets = semantic_reach_operation.discard_nodes_with_short_edge(reachable_sets,
                                                                                    self.config.reachable_set.length_edge_node_min)
            if step != self.step_start:
                # this sets the correct step for the new reach nodes ...
                reachable_sets = reach_operation.connect_children_to_parents(step, reachable_sets)
            else:
                # ... so we need to do this manually for the initial step, as there are no parents here
                for node in reachable_sets:
                    node.step = step

            # assign label to all newly constructed reach nodes
            _, target_states = automaton_states
            for node in reachable_sets:
                self.reachable_set_to_label[node] = target_states
            new_reachable_sets += reachable_sets

        self.dict_step_to_reachable_set[step] = new_reachable_sets

    def _split_reachable_set(self, step: int, reachable_set: ReachNode) -> List[ReachNode]:
        """Split the given reachable set along the transitions of the automaton states of its propagation source.

        For this, consider the outgoing transitions of all automaton states in the labels of the propagation source.
        We then split and cut the reachable set along the conditions of these transitions.
        :param step: Current step of the reachability analysis.
        :param reachable_set: The reachable set to split.
        :return: List of reachable sets so that each is a subset of the given reachable set,
            and satisfies the condition of at least one transition (up to overapproximation).
        """
        current_states = frozenset({self.automaton.initial_state}) if step == self.step_start else \
            self.reachable_set_to_label[reachable_set.source_propagation]
        transitions = list(self.automaton.transitions_from(current_states))
        # TODO: use splitter here and assign labels correctly
        constrained_reachable_sets = self._split_to_minterms(step, [reachable_set], transitions)

        constrained_reachable_sets = self._filter_reachable_sets(constrained_reachable_sets, step)

        constrained_reachable_sets = self._deduplicate_reachable_sets(constrained_reachable_sets)

        return constrained_reachable_sets

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

    def _deduplicate_reachable_sets(self, constrained_reachable_sets: List[ReachNode]) -> List[ReachNode]:
        """Deduplicate reachable sets and merge labels of duplicates."""
        unique_reachable_sets = []
        for reachable_set in constrained_reachable_sets:
            for other in unique_reachable_sets:
                # Two reachable sets are equal to us, if both their lat and lon polygons are equal
                equal_lon = reachable_set.polygon_lon.shapely_object.equals(other.polygon_lon.shapely_object)
                equal_lat = reachable_set.polygon_lat.shapely_object.equals(other.polygon_lat.shapely_object)
                if equal_lon and equal_lat:
                    other_labels = self.reachable_set_to_label[other]
                    self.reachable_set_to_label[other] = other_labels.union(self.reachable_set_to_label[reachable_set])
                    break
            else:
                unique_reachable_sets.append(reachable_set)
        return unique_reachable_sets
