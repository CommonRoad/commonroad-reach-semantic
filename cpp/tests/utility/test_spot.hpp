#pragma once

#include "reach_semantic/utility/spot.hpp"

#include <gtest/gtest.h>
#include <spot/tl/formula.hh>
#include <spot/tl/parse.hh>

class SpotUtilityTestInitialization {
protected:
    spot::formula conjunction;
    spot::formula disjunction;
    spot::formula implication;
    spot::formula negated_conjunction;
    spot::formula positive_literal;
    spot::formula negative_literal;
    spot::formula true_formula;
    spot::formula false_formula;
    spot::formula dnf;

    void set_up_formulas();
};

class SpotUtilityTest : public SpotUtilityTestInitialization, public testing::Test {
private:
    void SetUp() override;
};
