#pragma once

#include "reach_semantic/utility/spot.hpp"

#include <gtest/gtest.h>
#include <spot/tl/formula.hh>
#include <spot/tl/parse.hh>

class SpotUtilityTest : public testing::Test {
private:
    void SetUp() override;
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
};
