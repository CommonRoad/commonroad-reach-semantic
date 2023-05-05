import logging
from collections import defaultdict
from typing import List, Dict, Set

from commonroad_reach_semantic.data_structure.config.semantic_configuration import SemanticConfiguration
from commonroad_reach_semantic.data_structure.reach.semantic_reach_node import SemanticReachNode
from commonroad_reach_semantic.data_structure.rule.proposition_holder import PropositionHolder

logger = logging.getLogger(__name__)


class TPLChecker:
    """Stores TPL specifications and checks reachable sets against them."""

    set_identifiers_tpl: Set[str] = {"(", ")", "->", "<->", "&", "|", "!", "xor"}

    config: SemanticConfiguration
    list_specifications_tpl: List[str]
    dict_step_to_propositions_mandatory: Dict[int, Set[str]]
    dict_step_to_propositions_forbidden: Dict[int, Set[str]]

    def __init__(self, config: SemanticConfiguration) -> None:
        self.config = config
        self.list_specifications_tpl = list()

        # mandatory and forbidden propositions
        self.dict_step_to_propositions_mandatory = defaultdict(set)
        self.dict_step_to_propositions_forbidden = defaultdict(set)

    def add_specification(self, specification: str) -> None:
        """
        Stores a new TPL specification.
        """
        self.list_specifications_tpl.append(specification)

    def extract_mandatory_and_forbidden_propositions(self) -> None:
        """
        Extracts and stores mandatory and forbidden propositions into dictionaries.
        """
        for specification in self.list_specifications_tpl:
            index_bracket_left = specification.find("[")
            index_bracket_right = specification.find("]")
            # if time step is indicated in the specification
            if index_bracket_left >= 0 and index_bracket_right >= 0:
                step_start = int(specification[index_bracket_left + 1:index_bracket_right])
                step_end = step_start
                specification = specification[index_bracket_right + 2:]

            else:
                step_start = self.config.planning.step_start
                step_end = step_start + self.config.planning.steps_computation

            set_clauses = specification.split(" & ")
            for clause in set_clauses:
                if clause[0] != "!":
                    for step in range(step_start, step_end + 1):
                        self.dict_step_to_propositions_mandatory[step].add(clause)

                else:
                    for step in range(step_start, step_end + 1):
                        self.dict_step_to_propositions_forbidden[step].add(clause[1:])

        logger.info("Mandatory and forbidden propositions extracted.")

    def examine_tpl_specifications(self, step: int, list_propagated_sets: List[SemanticReachNode],
                                   reachable_set_to_propositions: Dict[SemanticReachNode, PropositionHolder]) -> List[
        SemanticReachNode]:
        """
        Examines whether the given propagated sets satisfy the TPL specifications.
        """
        set_propositions_mandatory = self.dict_step_to_propositions_mandatory[step]
        set_propositions_forbidden = self.dict_step_to_propositions_forbidden[step]

        list_propagated_sets_keep = [propagated_set for propagated_set in list_propagated_sets
                                     if reachable_set_to_propositions[propagated_set].set_propositions.issuperset(
                set_propositions_mandatory)]
        list_propagated_sets_keep = [propagated_set for propagated_set in list_propagated_sets_keep
                                     if reachable_set_to_propositions[propagated_set].set_propositions.isdisjoint(
                set_propositions_forbidden)]

        return list_propagated_sets_keep
