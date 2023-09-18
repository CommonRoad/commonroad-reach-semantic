#include "reach_semantic/utility/spot.hpp"

#include <doctest/doctest.h>
#include <spot/tl/formula.hh>
#include <spot/tl/parse.hh>

TEST_SUITE("Spot utility") {
  TEST_CASE("disjuncts") {
    spot::formula formula;
    std::vector<spot::formula> expected;
    SUBCASE("conjunction") {
      formula = spot::parse_formula("a & !b & c");
      expected = {spot::parse_formula("a & !b & c")};
    }
    SUBCASE("DNF") {
      formula = spot::parse_formula("(a & !b) | (!c & d & e)");
      expected = {spot::parse_formula("a & !b"),
                  spot::parse_formula("!c & d & e")};
    }

    auto actual = util_spot::disjuncts(formula);
    CHECK_EQ(actual.size(), expected.size());
    for (auto i = 0; i < actual.size(); ++i) {
      CHECK_EQ(actual[i], expected[i]);
    }
  }

  TEST_CASE("conjuncts") {
    spot::formula formula;
    std::vector<spot::formula> expected;
    SUBCASE("conjunction") {
      formula = spot::parse_formula("a & !b & c");
      expected = {spot::parse_formula("a"), spot::parse_formula("!b"),
                  spot::parse_formula("c")};
    }
    SUBCASE("DNF") {
      formula = spot::parse_formula("(a & !b) | (!c & d & e)");
      expected = {spot::parse_formula("(a & !b) | (!c & d & e)")};
    }

    auto actual = util_spot::conjuncts(formula);
    CHECK_EQ(actual.size(), expected.size());
    for (auto i = 0; i < actual.size(); ++i) {
      CHECK_EQ(actual[i], expected[i]);
    }
  }

  TEST_CASE("extract_atomic_proposition") {
    spot::formula literal;
    std::pair<std::string, bool> expected;
    SUBCASE("positive") {
      literal = spot::parse_formula("a");
      expected = {"a", false};
    }
    SUBCASE("negative") {
      literal = spot::parse_formula("!a");
      expected = {"a", true};
    }

    auto actual = util_spot::extract_atomic_proposition(literal);
    CHECK_EQ(actual, expected);
  }

  TEST_CASE("extract_atomic_propositions raises") {
    spot::formula non_literal;
    SUBCASE("true") { non_literal = spot::parse_formula("true"); }
    SUBCASE("false") { non_literal = spot::parse_formula("false"); }
    SUBCASE("conjunction") { non_literal = spot::parse_formula("a & b"); }
    SUBCASE("disjunction") { non_literal = spot::parse_formula("a | b"); }

    CHECK_THROWS_AS(util_spot::extract_atomic_proposition(non_literal),
                    std::invalid_argument);
  }

  TEST_CASE("extract_minterms_from_dnf") {
    spot::formula dnf_formula;
    std::vector<std::set<std::pair<std::string, bool>>> expected;
    SUBCASE("true") {
      dnf_formula = spot::parse_formula("true");
      expected = {{}};
    }
    SUBCASE("false") {
      dnf_formula = spot::parse_formula("false");
      expected = {};
    }
    SUBCASE("positive") {
      dnf_formula = spot::parse_formula("a");
      expected = {{{"a", false}}};
    }
    SUBCASE("negative") {
      dnf_formula = spot::parse_formula("!a");
      expected = {{{"a", true}}};
    }
    SUBCASE("conjunction") {
      dnf_formula = spot::parse_formula("a & !b & c");
      expected = {{{"a", false}, {"b", true}, {"c", false}}};
    }
    SUBCASE("disjunction") {
      dnf_formula = spot::parse_formula("a | !b");
      expected = {{{"a", false}}, {{"b", true}}};
    }
    SUBCASE("DNF") {
      dnf_formula = spot::parse_formula("(a | b) | c");
      expected = {{{"a", false}}, {{"b", false}}, {{"c", false}}};
    }
    SUBCASE("DNF with conjunctions") {
      dnf_formula = spot::parse_formula("(a & !b) | (!c & d & e)");
      expected = {{{"a", false}, {"b", true}},
                  {{"c", true}, {"d", false}, {"e", false}}};
    }

    auto actual = util_spot::extract_minterms_from_dnf(dnf_formula);
    CHECK_EQ(actual.size(), expected.size());
    for (auto i = 0; i < actual.size(); ++i) {
      CHECK_EQ(actual[i], expected[i]);
    }
  }

  TEST_CASE("extract_minterms_from_dnf raises") {
    spot::formula non_dnf_formula;
    SUBCASE("conjunction") {
      non_dnf_formula = spot::parse_formula("(a | b) & c");
    }
    SUBCASE("negated conjunction") {
      non_dnf_formula = spot::parse_formula("a | !(b & c)");
    }
    SUBCASE("implication") { non_dnf_formula = spot::parse_formula("a -> b"); }

    CHECK_THROWS_AS(util_spot::extract_minterms_from_dnf(non_dnf_formula),
                    std::invalid_argument);
  }
}
