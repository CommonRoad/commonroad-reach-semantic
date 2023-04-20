from __future__ import annotations

import copy
from collections import defaultdict
from functools import lru_cache
from typing import Set, Dict

from commonroad_reach_semantic_addon.data_structure.proposition import PropositionGroup as PG


class PropositionHolder:
    dict_group_to_set_propositions: Dict[PG, Set[str]]

    def __init__(self, set_propositions: Set[str] = None, group=None):
        self.dict_group_to_set_propositions = defaultdict(set)
        self.set_propositions = {"true"}
        self.set_propositions_temporary = set()

        if set_propositions and group:
            self.add_propositions(set_propositions, group)

    def __key(self):
        # temporary propositions are not hashed!
        return frozenset(self.set_propositions)

    def __hash__(self):
        return hash(self.__key())

    def __eq__(self, other):
        if isinstance(other, PropositionHolder) and self.__key() == other.__key():
            return True

        else:
            return False

    def __repr__(self):
        return f"PropHolder(POS={self.propositions_in_group(group=PG.POSITION)})"

    def clone(self):
        holder_cloned = PropositionHolder()
        holder_cloned.set_propositions = copy.deepcopy(self.set_propositions)
        holder_cloned.set_propositions_temporary = copy.deepcopy(self.set_propositions_temporary)
        holder_cloned.dict_group_to_set_propositions = copy.deepcopy(self.dict_group_to_set_propositions)

        return holder_cloned

    @lru_cache(maxsize=500)
    def propositions(self, include_temporary: bool = True) -> Set[str]:
        if include_temporary:
            return self.set_propositions.union(self.set_propositions_temporary)

        else:
            return self.set_propositions

    def propositions_in_group(self, group: PG):
        return self.dict_group_to_set_propositions[group]

    def add_propositions(self, propositions: Set[str], group: PG):
        self.dict_group_to_set_propositions[group].update(propositions)

        if group == PG.TEMPORARY:
            self.set_propositions_temporary.update(propositions)
        else:
            self.set_propositions.update(propositions)

    def add_proposition(self, proposition: str, group: PG):
        self.dict_group_to_set_propositions[group].add(proposition)

        if group == PG.TEMPORARY:
            self.set_propositions_temporary.add(proposition)
        else:
            self.set_propositions.add(proposition)

    def merge(self, other: PropositionHolder):
        for group, props in other.dict_group_to_set_propositions.items():
            self.add_propositions(props, group)


class MultiStepPropositionHolder:
    def __init__(self, step_end: int):
        self.set_propositions_time_invariant = set()
        self.dict_step_to_set_propositions_time_variant = {step: set() for step in range(step_end)}

        self.dict_group_to_set_propositions_time_invariant = defaultdict(set)
        self.dict_group_to_step_to_set_propositions_time_variant = defaultdict(dict)

    def time_invariant_propositions(self, group: PG = None):
        if not group:
            return self.set_propositions_time_invariant

        else:
            return self.dict_group_to_set_propositions_time_invariant[group]

    def time_variant_propositions(self, group: PG = None):
        if not group:
            return self.dict_step_to_set_propositions_time_variant

        else:
            return self.dict_group_to_step_to_set_propositions_time_variant[group]

    def time_variant_propositions_at_step(self, step: int, group: PG = None):
        if not group:
            return self.dict_step_to_set_propositions_time_variant[step]

        else:
            return self.dict_group_to_step_to_set_propositions_time_variant[group][step]

    def propositions_at_step(self, step: int, group: PG = None):
        set_proposition = set()
        if not group:
            set_proposition.update(self.set_propositions_time_invariant)
            set_proposition.update(self.dict_step_to_set_propositions_time_variant[step])

        else:
            set_proposition.update(self.dict_group_to_set_propositions_time_invariant[group])
            set_proposition.update(self.dict_group_to_step_to_set_propositions_time_variant[group][step])

        return set_proposition

    def dict_group_to_propositions_at_step(self, step: int):
        dict_group_to_set_propositions = defaultdict(set)
        for group, set_propositions in self.dict_group_to_set_propositions_time_invariant.items():
            dict_group_to_set_propositions[group].update(set_propositions)

        for group, dict_step_to_set_propositions in self.dict_group_to_step_to_set_propositions_time_variant.items():
            set_propositions = dict_step_to_set_propositions[step]
            dict_group_to_set_propositions[group].update(set_propositions)

        return dict_group_to_set_propositions

    def add_proposition(self, proposition, group: PG, step: int = -1):
        if step == -1:
            self.set_propositions_time_invariant.add(proposition)
            self.dict_group_to_set_propositions_time_invariant[group].add(proposition)

        else:
            self.dict_step_to_set_propositions_time_variant[step].add(proposition)
            try:
                self.dict_group_to_step_to_set_propositions_time_variant[group][step].add(proposition)

            except KeyError:
                self.dict_group_to_step_to_set_propositions_time_variant[group][step] = {proposition}
