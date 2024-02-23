#pragma once

#include "../predicate_test_setup.hpp"
#include "reach_semantic/data_structure/reach/predicates/braking/keeps_safe_distance_prec_predicate.hpp"
#include "reachset/data_structure/reach/reach_node.hpp"

#include <gtest/gtest.h>

class KeepsSafeDistancePrecPredicateTest : public testing::Test {
  private:
    void SetUp() override;

  protected:
    static constexpr double tolerance = 1e-6;
    static constexpr double overapprox_tolerance = 0.5;

    static constexpr double ego_length = 4.5;
    static constexpr double ego_reaction_time = 0.3;
    static constexpr double ego_deceleration = -10.0;
    static constexpr double other_deceleration = -10.5;

    TestEnvironments test_envs;

    std::unique_ptr<semantic_reach::KeepsSafeDistancePrecPredicate> positive_pred;
    std::unique_ptr<semantic_reach::KeepsSafeDistancePrecPredicate> negative_pred;

    std::shared_ptr<reach::ReachNode> reach_node_one;

    static double safe_position_world_one(double ego_velocity);

    static bool contains_point(const reach::ReachPolygon &polygon, double position, double velocity);
};
