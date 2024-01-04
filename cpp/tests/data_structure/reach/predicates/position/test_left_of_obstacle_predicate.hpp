#pragma once

#include "../predicate_test_setup.hpp"
#include "reach_semantic/data_structure/reach/predicates/position/left_of_obstacle_predicate.hpp"
#include "reachset/data_structure/reach/reach_node.hpp"

#include <gtest/gtest.h>

class LeftOfObstaclePredicateTest : public testing::Test {
  private:
    void SetUp() override;

  protected:
    static constexpr double tolerance = 1e-6;

    static constexpr double ego_length = 4.5;

    TestEnvironments test_envs;

    std::unique_ptr<semantic_reach::LeftOfObstaclePredicate> positive_pred;
    std::unique_ptr<semantic_reach::LeftOfObstaclePredicate> negative_pred;

    std::shared_ptr<reach::ReachNode> reach_node_one;
};
