import more_itertools
import logging
from collections import defaultdict

from commonroad_reach.utility import reach_operation

import commonroad_reach_semantic.utility.reach_operation as semantic_reach_operation
from commonroad_reach_semantic.data_structure.config.semantic_configuration import SemanticConfiguration
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.reach.reachable_set_labeler import ReachableSetLabeler
from commonroad_reach_semantic.data_structure.reach.semantic_reach_set_py import PySemanticReachableSet
from commonroad_reach_semantic.data_structure.rule.traffic_rule_interface import TrafficRuleInterface

logger = logging.getLogger(__name__)


class PySemanticLabelingReachableSet(PySemanticReachableSet):
    """
    Reachable set computation considering temporal constraints with Python backend.
    """

    config: SemanticConfiguration

    def __init__(self, config: SemanticConfiguration, semantic_model: SemanticModel,
                 rule_interface: TrafficRuleInterface):
        super().__init__(config, semantic_model, rule_interface)
        self.labeler = ReachableSetLabeler(semantic_model)
        self.dict_step_to_reachable_set[self.step_start] = self._construct_initial_reachable_sets()
        self.dict_step_to_drivable_area[self.step_start] = reach_operation.project_propagated_sets_to_position_domain(
            self.dict_step_to_reachable_set[self.step_start])

        self.dict_step_to_propositions_to_drivable_area = dict()
        self.dict_step_to_propositions_to_propagated_set = dict()

        self.labeler.label_initial_state(self.dict_step_to_reachable_set[self.step_start], self.step_start)

        logger.debug("PySemanticLabelingReachableSet initialized.")

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
            self.dict_step_to_propositions_to_drivable_area[step] = dict()
            self.dict_step_to_propositions_to_propagated_set[step] = dict()
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

        # partition propagated sets by their propositions
        dict_propositions_to_propagated_set = defaultdict(list)
        for propagated_set in propagated_sets:
            dict_propositions_to_propagated_set[self.labeler.reachable_set_to_propositions[propagated_set]].append(
                propagated_set)

        # merge, collision check, and repartition propagated sets partitioned by their propositions,
        # because we must not merge sets with different propositions
        dict_propositions_to_drivable_area = dict()
        for propositions, propagated_sets_per_proposition in dict_propositions_to_propagated_set.items():
            list_rectangles_projected = reach_operation.project_propagated_sets_to_position_domain(
                propagated_sets_per_proposition)
            dict_propositions_to_drivable_area[propositions] = self._collision_check_and_repartition(
                list_rectangles_projected, step)

        self.dict_step_to_drivable_area[step] = list(
            more_itertools.flatten(dict_propositions_to_drivable_area.values()))
        self.dict_step_to_propositions_to_drivable_area[step] = dict_propositions_to_drivable_area
        self.dict_step_to_propositions_to_propagated_set[step] = dict_propositions_to_propagated_set
        self.dict_step_to_propagated_set[step] = propagated_sets

    def _compute_reachable_set_at_step(self, step):
        """
        Computes reachable set for the given step.

        Steps:
            1. construct reach nodes from drivable area and the propagated sets.
            2. update parent-child relationship of the nodes.
        """
        dict_propositions_to_propagated_set = self.dict_step_to_propositions_to_propagated_set[step]
        dict_propositions_to_drivable_area = self.dict_step_to_propositions_to_drivable_area[step]

        if not dict_propositions_to_drivable_area:
            self.dict_step_to_reachable_set[step] = list()
            return None

        # discard drivable area with small area if there are more than one node (this is subject to change)
        num_drivable_area = sum(
            [len(list_drivable) for list_drivable in dict_propositions_to_drivable_area.values()])
        discard_small_node = (num_drivable_area > 1) and self.config.reachable_set.discard_small_nodes

        # work with the reachable sets partitioned by propositions here, because otherwise it could happen
        # that we merge two reachable sets with different propositions when they intersect with the same drivable area
        dict_propositions_to_reachable_set = dict()
        for proposition_holder, drivable_area in dict_propositions_to_drivable_area.items():
            propagated_sets = dict_propositions_to_propagated_set[proposition_holder]

            list_nodes = reach_operation.construct_reach_nodes(drivable_area, propagated_sets)
            if discard_small_node:
                list_nodes = semantic_reach_operation.discard_nodes_with_short_edge(list_nodes,
                                                                                    self.config.reachable_set.length_edge_node_min)
            if list_nodes:
                reachable_sets = reach_operation.connect_children_to_parents(step, list_nodes)
                # copy propositions for newly constructed nodes. Because all propagated sets are labeled with the same
                # propositions, we simply use the first as reference.
                self.labeler.copy_labels(propagated_sets[0], *reachable_sets)
                dict_propositions_to_reachable_set[proposition_holder] = reachable_sets

        self.dict_step_to_reachable_set[step] = list(
            more_itertools.flatten(dict_propositions_to_reachable_set.values()))
