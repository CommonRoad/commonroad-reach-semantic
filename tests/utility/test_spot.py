import pytest
import spot

from commonroad_reach_semantic.utility import spot as util_spot


class TestSpotUtility:

    @pytest.mark.parametrize("formula, expected", [
        (spot.formula("a & !b & c"), [spot.formula("a & !b & c")]),
        (spot.formula("(a & !b) | (!c & d & e)"), [spot.formula("a & !b"), spot.formula("!c & d & e")])
    ])
    def test_disjuncts(self, formula, expected):
        for actual_child, expected_child in zip(util_spot.disjuncts(formula), expected):
            assert actual_child == expected_child

    @pytest.mark.parametrize("formula, expected", [
        (spot.formula("a & !b & c"), [spot.formula("a"), spot.formula("!b"), spot.formula("c")]),
        (spot.formula("(a & !b) | (!c & d & e)"), [spot.formula("(a & !b) | (!c & d & e)")])
    ])
    def test_conjuncts(self, formula, expected):
        for actual_child, expected_child in zip(util_spot.conjuncts(formula), expected):
            assert actual_child == expected_child

    @pytest.mark.parametrize("literal, expected", [
        (spot.formula("a"), ("a", False)),
        (spot.formula("!a"), ("a", True)),
    ])
    def test_extract_atomic_proposition(self, literal, expected):
        assert util_spot.extract_atomic_proposition(literal) == expected

    @pytest.mark.parametrize("non_literal", [
        spot.formula("true"),
        spot.formula("false"),
        spot.formula("a & b"),
        spot.formula("a | b"),
    ])
    def test_extract_atomic_proposition_raises(self, non_literal):
        with pytest.raises(ValueError, match=r"not a \(negated\) literal"):
            util_spot.extract_atomic_proposition(non_literal)

    @pytest.mark.parametrize("dnf_formula, expected", [
        (spot.formula("true"), [[]]),
        (spot.formula("false"), []),
        (spot.formula("a"), [[("a", False)]]),
        (spot.formula("!a"), [[("a", True)]]),
        (spot.formula("a & !b & c"), [[("a", False), ("b", True), ("c", False)]]),
        (spot.formula("a | !b"), [[("a", False)], [("b", True)]]),
        (spot.formula("(a | b) | c"), [[("a", False)], [("b", False)], [("c", False)]]),
        (spot.formula("(a & !b) | (!c & d & e)"), [[("a", False), ("b", True)], [("c", True), ("d", False), ("e", False)]]),
    ])
    def test_extract_minterms_from_dnf(self, dnf_formula, expected):
        assert util_spot.extract_minterms_from_dnf(dnf_formula) == expected

    @pytest.mark.parametrize("non_dnf_formula", [
        spot.formula("(a | b) & c"),
        spot.formula("a | !(b & c)"),
        spot.formula("a -> b"),
    ])
    def test_extract_minterms_from_dnf_raises(self, non_dnf_formula):
        with pytest.raises(ValueError, match=r"not in DNF"):
            util_spot.extract_minterms_from_dnf(non_dnf_formula)

