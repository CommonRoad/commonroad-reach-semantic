import functools
from collections import defaultdict
from typing import Iterator, List, Dict, Iterable

import buddy
import spot

import commonroad_reach_semantic.utility.spot as util_spot


class FiniteAutomaton:
    """Represents a finite automaton on words over the powerset of propositions."""

    _spot_automaton: spot.twa_graph
    _bdict: spot.bdd_dict

    def __init__(self, ltlf_formula: str) -> None:
        """Construct the finite automaton accepting those words that make the given LTLf formula true

        :param ltlf_formula: The LTLf formula to translate.
        """
        # disable simulation based reductions to speed up translation
        # see https://spot.lre.epita.fr/man/spot-x.7.html
        buechi_automaton = spot.from_ltlf(ltlf_formula).translate(xargs="simul=0")
        self._spot_automaton = spot.to_finite(buechi_automaton)
        self._bdict = self._spot_automaton.get_dict()

    @property
    def initial_state(self) -> int:
        """The number of the initial state."""
        return self._spot_automaton.get_init_state_number()

    def transitions_from(self, state: int) -> Iterator[tuple[int, List[List[tuple[str, bool]]]]]:
        """Iterate over all transitions outgoing from the given state."""
        for edge in self._spot_automaton.out(state):
            yield edge.dst, self._edge_condition_to_minterms(edge.cond)

    def combined_transitions_from(self, states: Iterable[int]) -> Iterator[tuple[int, List[List[tuple[str, bool]]]]]:
        """Iterate over all transitions outgoing from the given states.

        Tries to minimize the minterms by combining the conditions of the outgoing edges leading to the same destination.
        """
        dst_state_to_conditions: Dict[int, List[buddy.bdd]] = defaultdict(list)
        for state in frozenset(states):
            for edge in self._spot_automaton.out(state):
                dst_state_to_conditions[edge.dst].append(edge.cond)
        for dst_state, conditions in dst_state_to_conditions.items():
            yield dst_state, self._edge_condition_to_minterms(functools.reduce(buddy.bdd_or, conditions))

    def is_accepting_state(self, state: int) -> bool:
        """Check whether the given state is an accepting state.

        :returns: True if and only if the state is accepting.
        """
        return self._spot_automaton.state_is_accepting(state)

    def _edge_condition_to_minterms(self, cond: buddy.bdd) -> List[List[tuple[str, bool]]]:
        """Convert a condition on an automaton edge given as a BDD into a list of minterms.

        The condition is true iff at least one minterm is satisfied.
        A minterm is a list of possible negated atomic propositions.
        It is satisfied iff all its atomic propositions hold.
        """
        # will be in DNF --> bbd_to_formula computes an irredundant sum of products
        # https://spot.lre.epita.fr/doxygen/namespacespot.html#aba9b9efe994006c29a6d77da94897df8
        formula_dnf = spot.bdd_to_formula(cond, self._bdict)
        return util_spot.extract_minterms_from_dnf(formula_dnf)
