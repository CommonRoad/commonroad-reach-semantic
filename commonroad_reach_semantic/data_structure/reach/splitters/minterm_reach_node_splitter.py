from collections import Counter
from typing import List, Tuple, Iterable, Optional, Dict, FrozenSet

import more_itertools
from commonroad_reach.data_structure.reach.reach_node import ReachNode

from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.model_checking.finite_automaton import Minterm, Literal
from commonroad_reach_semantic.data_structure.reach import predicates
from commonroad_reach_semantic.data_structure.reach.reachable_set_labeler import ReachableSetLabeler


class MintermReachNodeSplitter:
    """Splits reach nodes according to given minterms."""

    labeler: ReachableSetLabeler

    def __init__(self, semantic_model: SemanticModel) -> None:
        """Create a new minterm reach node splitter.

        :param semantic_model: The semantic model to use for splitting.
        """
        self.labeler = ReachableSetLabeler(semantic_model)

    def split_to_minterms(self, step: int, reachable_set: ReachNode, minterms: Iterable[Minterm]) -> \
            Dict[Minterm, List[ReachNode]]:
        """Split the given reachable set along the given minterms.

        :param step: Current step of the reachability analysis.
        :param reachable_set: Reachable set to split.
        :param minterms: Minterms to split along.
        :return: List of reachable sets for each given minterm.
            The union of the reachable sets associated with a minterm overapproximates the original reachable set
            restricted to states that satisfy the minterm.
            A minterm may be omitted if the corresponding restricted set is empty.
        """
        result = {}
        # remove duplicates
        minterms = frozenset(minterms)
        self._split_to_minterms(step, [reachable_set], minterms, frozenset(), False, result)
        return result

    def _split_to_minterms(self, step: int, reachable_sets: List[ReachNode],
                           minterms: FrozenSet[Minterm],
                           finished_literals: FrozenSet[Literal], regionized: bool,
                           result: Dict[Minterm, List[ReachNode]]) -> None:
        """Implementation of split_to_minterms.

        This is a recursive function that splits the reachable sets along the given minterms.
        The goal is to reuse as many splits as possible.
        First, we select a (not yet finished) literal that we will use for splitting in this step.
        Then, we partition the minterms into those that depend on the literal and those that do not.
        To further handle the former, we need to restrict the reachable sets to the chosen literal.
        The chosen literal is marked as finished for the restricted reachable sets.
        Thus, when _split_to_minterms is called, all nodes in reachable_sets satisfy all literals in finished_literals.
        We then recursively split the original reachable sets along the minterms that do not depend on the literal,
        and the restricted reachable sets along the minterms that do depend on the literal.
        The recursion ends, when there are no more reachable sets, because restricting them along the literal resulted
        in an empty set.
        The recursion also ends, when we considered all literals.

        :param step: Current step of the reachability analysis.
        :param reachable_sets: Reachable sets to split.
        :param minterms: Minterms to split along.
        :param finished_literals: Literals that we no longer have to consider.
        :param regionized: Whether the reachable sets are already split into regions.
        :return: List of reachable sets for each given minterm.
            The union of the reachable sets associated with a minterm overapproximates the original reachable sets
            restricted to states that satisfy the minterm.
        """
        # BASE CASE: if there are no reachable sets or no minterms, we are done
        if not reachable_sets or not minterms:
            return

        # select the next literal to split on
        literal_to_split = self._choose_next_literal(minterms, finished_literals)

        # BASE CASE: if there is no literal to split, we are done
        if not literal_to_split:
            # if we did not find a literal to split, all remaining minterms are the same.
            assert len(minterms) == 1
            # moreover, the literals in the remaining minterms are all finished.
            assert next(minterms.__iter__()) == finished_literals
            # write the reachable sets to the result
            result[finished_literals] = reachable_sets
            return

        # partition the minterms into those that contain the literal and those that don't
        not_has_literal, has_literal = self._partition_minterms(literal_to_split, minterms)

        # if there are minterms that don't contain the current literal,
        # we have to clone the reach nodes before restricting
        # so that we can keep the original nodes for those minterms
        restricted_reachable_sets, restriction_regionized = self._restrict_to_literal(step, reachable_sets,
                                                                                      literal_to_split, regionized,
                                                                                      clone=bool(not_has_literal))

        # recurse to split along the remaining literals
        # note that only the restricted reachable sets might have been regionized
        self._split_to_minterms(step, reachable_sets, not_has_literal, finished_literals, regionized, result)
        # argument has to be a one-tuple here, otherwise the literal (which is a tuple itself) will be unpacked
        new_finished_literals = finished_literals.union((literal_to_split,))
        self._split_to_minterms(step, restricted_reachable_sets, has_literal, new_finished_literals,
                                regionized or restriction_regionized, result)

    def _restrict_to_literal(self, step: int, reachable_sets: List[ReachNode], literal: Literal,
                             regionized: bool, clone: bool = True) -> Tuple[List[ReachNode], bool]:
        """Restrict the reachable sets to the given literal.

        If restricting requires lanelet information, we first split the reachable sets into regions.
        If we clone the reachable sets, we also copy the labels from the original nodes to the clones.
        :param step: Current step of the reachability analysis.
        :param reachable_sets: The reachable sets to restrict.
        :param literal: The literal to restrict to.
        :param regionized: Whether the reachable sets are already split into regions.
        :param clone: Whether to clone the reachable sets before restricting.
        :return: The restricted reachable sets and whether they were split into regions.
        """
        to_restrict = [node.clone() for node in reachable_sets] if clone else reachable_sets

        # if we already split to regions, we need to copy labels from the original nodes to the clones
        if clone and regionized:
            for src, dst in zip(reachable_sets, to_restrict):
                self.labeler.copy_labels(src, dst)

        pred = predicates.from_proposition(*literal)

        # if the predicate needs lanelets,
        # we need to split the reachable sets into regions first (if we haven't already)
        if pred.needs_lanelets and not regionized:
            to_restrict = list(more_itertools.flatten(
                self.labeler.split_wrt_regions(step, node)
                for node in to_restrict
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
    def _partition_minterms(literal: Literal, minterms: Iterable[Minterm]) -> Tuple[
        FrozenSet[Minterm], FrozenSet[Minterm]]:
        """Partition the minterms into those that contain the literal and those that don't.

        :param literal: The literal to partition the minterms along.
        :param minterms: The minterms to partition.
        :return: A tuple of two minterm lists, the first containing the minterms don't contain the literal,
            the second containing the mintrems that do.
        """
        not_needs_literal, needs_literal = more_itertools.partition(lambda m: literal in m, minterms)
        return frozenset(not_needs_literal), frozenset(needs_literal)

    @staticmethod
    def _choose_next_literal(minterms: Iterable[Minterm],
                             ignored_literals: Iterable[Literal]) -> Optional[Literal]:
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
        candidates = (literal for literal, cnt in c.items() if cnt == max_cnt)

        # prefer predicates that don't need lanelets, as this avoids splitting to regions
        # TODO: we could choose a different ordering here or make this configurable
        candidates = sorted(candidates, key=lambda literal: predicates.from_proposition(*literal).needs_lanelets)

        return next(candidates.__iter__(), None)
