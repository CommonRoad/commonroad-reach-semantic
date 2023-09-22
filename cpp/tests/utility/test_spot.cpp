#include "test_spot.hpp"

void SpotUtilityTest::SetUp() {
    conjunction = spot::parse_formula("a & !b & c");
    disjunction = spot::parse_formula("a | !b | c");
    implication = spot::parse_formula("a -> b");
    negated_conjunction = spot::parse_formula("!(a & b & c)");
    positive_literal = spot::parse_formula("a");
    negative_literal = spot::parse_formula("!a");
    true_formula = spot::parse_formula("true");
    false_formula = spot::parse_formula("false");
    dnf = spot::parse_formula("(a & !b) | (!c & d & e)");
}

TEST_F(SpotUtilityTest, Disjuncts) {
    std::vector<spot::formula> expected{conjunction};
    EXPECT_EQ(util_spot::disjuncts(conjunction), expected);
    expected = {spot::parse_formula("a"), spot::parse_formula("!b"), spot::parse_formula("c")};
    EXPECT_EQ(util_spot::disjuncts(disjunction), expected);
}

TEST_F(SpotUtilityTest, Conjuncts) {
    std::vector<spot::formula> expected{spot::parse_formula("a"), spot::parse_formula("!b"), spot::parse_formula("c")};
    EXPECT_EQ(util_spot::conjuncts(conjunction), expected);
    expected = {disjunction};
    EXPECT_EQ(util_spot::conjuncts(disjunction), expected);
}

TEST_F(SpotUtilityTest, ExtractAtomicPropositions) {
    std::pair<std::string, bool> expected{"a", false};
    EXPECT_EQ(util_spot::extract_atomic_proposition(positive_literal), expected);
    expected = {"a", true};
    EXPECT_EQ(util_spot::extract_atomic_proposition(negative_literal), expected);
}

TEST_F(SpotUtilityTest, ExtractAtomicPropositionsException) {
    for (const auto &formula :
         {conjunction, disjunction, implication, negated_conjunction, true_formula, false_formula}) {
        EXPECT_THROW(util_spot::extract_atomic_proposition(formula), std::invalid_argument);
    }
}

TEST_F(SpotUtilityTest, ExtractMintermsFromDNF) {
    std::vector<std::set<std::pair<std::string, bool>>> expected{{}};
    EXPECT_EQ(util_spot::extract_minterms_from_dnf(true_formula), expected);
    expected = {};
    EXPECT_EQ(util_spot::extract_minterms_from_dnf(false_formula), expected);
    expected = {{{"a", false}}};
    EXPECT_EQ(util_spot::extract_minterms_from_dnf(positive_literal), expected);
    expected = {{{"a", true}}};
    EXPECT_EQ(util_spot::extract_minterms_from_dnf(negative_literal), expected);
    expected = {{{"a", false}, {"b", true}, {"c", false}}};
    EXPECT_EQ(util_spot::extract_minterms_from_dnf(conjunction), expected);
    expected = {{{"a", false}}, {{"b", true}}, {{"c", false}}};
    EXPECT_EQ(util_spot::extract_minterms_from_dnf(disjunction), expected);
    expected = {{{"a", false}, {"b", true}}, {{"c", true}, {"d", false}, {"e", false}}};
    EXPECT_EQ(util_spot::extract_minterms_from_dnf(dnf), expected);
}

TEST_F(SpotUtilityTest, ExtractMintermsFromDNFException) {
    for (const auto &formula : {implication, negated_conjunction}) {
        EXPECT_THROW(util_spot::extract_minterms_from_dnf(formula), std::invalid_argument);
    }
}
