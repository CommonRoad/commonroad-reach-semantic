#pragma once

#include "../predicate_test_setup.hpp"
#include "reach_semantic/data_structure/reach/predicates/position/in_front_of_obstacle_predicate.hpp"
#include "reachset/data_structure/reach/reach_node.hpp"

#include <gtest/gtest.h>

class InFrontOfObstaclePredicateTest : public testing::Test {
  private:
    void SetUp() override;

  protected:
    static constexpr double tolerance = 1e-6;

    TestEnvironments test_envs;
    TestPredicateConfigs test_configs;

    std::unique_ptr<semantic_reach::InFrontOfObstaclePredicate> positive_pred;
    std::unique_ptr<semantic_reach::InFrontOfObstaclePredicate> negative_pred;

    std::shared_ptr<reach::ReachNode> reach_node_one;
};
