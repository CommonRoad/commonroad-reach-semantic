import functools
from collections import defaultdict
from typing import Iterator, List, Dict, Iterable, Tuple, FrozenSet

import buddy
import spot

import commonroad_reach_semantic.utility.spot as util_spot


class FiniteAutomaton:
    """Represents a finite automaton on words over the powerset of propositions."""

    _spot_automaton: spot.twa_graph
    _bdict: spot.bdd_dict

    def __init__(self, ltlf_formulas: List[str], mode: int = 0) -> None:
        """Construct the finite automaton accepting those words that make the given LTLf formula true

        :param ltlf_formulas: The LTLf formulas to translate.
        :param mode: How to combine the formulas into a single automaton:
            0 = decide automatically
            1 = translate individually and then compute the product automaton (faster)
            2 = form conjunction of the formulas and translate the result (probably smaller automaton)
        """
        if mode == 0:
            # We always use the product automaton for now
            mode = 1

        spot_formulas = [spot.from_ltlf(f) for f in ltlf_formulas]
        if mode == 1:
            buechi_automata = [self._translate_ltlf_to_buechi(f) for f in spot_formulas]
            product_automaton = functools.reduce(spot.product, buechi_automata)
            self._spot_automaton = spot.to_finite(product_automaton)
        elif mode == 2:
            conjunction = self._formula_conjunction(spot_formulas)
            self._spot_automaton = spot.to_finite(self._translate_ltlf_to_buechi(conjunction))
        else:
            raise ValueError("Invalid mode")

        self._bdict = self._spot_automaton.get_dict()

    @property
    def initial_state(self) -> int:
        """The number of the initial state."""
        return self._spot_automaton.get_init_state_number()

    def transitions_from(self, state: int) -> Iterator[Tuple[List[FrozenSet[Tuple[str, bool]]], int]]:
        """Iterate over all transitions outgoing from the given state."""
        for edge in self._spot_automaton.out(state):
            yield self._edge_condition_to_minterms(edge.cond), edge.dst

    def combined_transitions_from(self, states: Iterable[int]) -> Iterator[Tuple[FrozenSet[Tuple[str, bool]], int]]:
        """Iterate over all transitions outgoing from the given states.

        Tries to minimize the minterms by combining the conditions of the outgoing edges leading to the same destination.
        """
        dst_state_to_conditions: Dict[int, List[buddy.bdd]] = defaultdict(list)
        for state in frozenset(states):
            for edge in self._spot_automaton.out(state):
                dst_state_to_conditions[edge.dst].append(edge.cond)
        for dst_state, conditions in dst_state_to_conditions.items():
            for minterm in self._edge_condition_to_minterms(functools.reduce(buddy.bdd_or, conditions)):
                yield frozenset(minterm), dst_state

    def is_accepting_state(self, state: int) -> bool:
        """Check whether the given state is an accepting state.

        :returns: True if and only if the state is accepting.
        """
        return self._spot_automaton.state_is_accepting(state)

    def _edge_condition_to_minterms(self, cond: buddy.bdd) -> List[FrozenSet[Tuple[str, bool]]]:
        """Convert a condition on an automaton edge given as a BDD into a list of minterms.

        The condition is true iff at least one minterm is satisfied.
        A minterm is a list of possibly negated atomic propositions.
        It is satisfied iff all its atomic propositions hold.
        """
        # will be in DNF --> bbd_to_formula computes an irredundant sum of products
        # https://spot.lre.epita.fr/doxygen/namespacespot.html#aba9b9efe994006c29a6d77da94897df8
        formula_dnf = spot.bdd_to_formula(cond, self._bdict)
        return [frozenset(minterm) for minterm in util_spot.extract_minterms_from_dnf(formula_dnf)]

    @staticmethod
    def _formula_conjunction(formulas: List[spot.formula]) -> spot.formula:
        """Conjunction the given LTLf formulas."""
        return spot.formula.And(formulas)

    @staticmethod
    def _translate_ltlf_to_buechi(ltlf_formula: spot.formula) -> spot.twa_graph:
        """Translate the given LTLf formula into a Buechi automaton."""
        # disable simulation based reductions to speed up translation
        # see https://spot.lre.epita.fr/man/spot-x.7.html
        return ltlf_formula.translate(xargs="simul=0")
