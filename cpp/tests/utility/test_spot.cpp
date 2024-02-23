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
    EXPECT_THROW(util_spot::extract_atomic_proposition(conjunction), std::invalid_argument);
    EXPECT_THROW(util_spot::extract_atomic_proposition(disjunction), std::invalid_argument);
    EXPECT_THROW(util_spot::extract_atomic_proposition(implication), std::invalid_argument);
    EXPECT_THROW(util_spot::extract_atomic_proposition(negated_conjunction), std::invalid_argument);
    EXPECT_THROW(util_spot::extract_atomic_proposition(true_formula), std::invalid_argument);
    EXPECT_THROW(util_spot::extract_atomic_proposition(false_formula), std::invalid_argument);
}

TEST_F(SpotUtilityTest, ExtractMintermsFromDNF) {
    std::vector<semantic_reach::Minterm> expected{{}};
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
    EXPECT_THROW(util_spot::extract_minterms_from_dnf(implication), std::invalid_argument);
    EXPECT_THROW(util_spot::extract_minterms_from_dnf(negated_conjunction), std::invalid_argument);
}
