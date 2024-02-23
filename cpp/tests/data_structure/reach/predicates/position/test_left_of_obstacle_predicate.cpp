#include "test_left_of_obstacle_predicate.hpp"

void LeftOfObstaclePredicateTest::SetUp() {
    test_envs.set_up_environments();
    positive_pred =
        std::make_unique<semantic_reach::LeftOfObstaclePredicate>(false, TestEnvironments::id_obstacle_one, ego_width);
    negative_pred =
        std::make_unique<semantic_reach::LeftOfObstaclePredicate>(true, TestEnvironments::id_obstacle_one, ego_width);

    reach_node_one = std::make_shared<reach::ReachNode>(
        TestEnvironments::time_step,
        std::make_shared<reach::ReachPolygon>(std::vector{std::tuple{0.0, 0.0}, {50.0, 0.0}, {50.0, 5.0}, {0.0, 5.0}}),
        std::make_shared<reach::ReachPolygon>(std::vector{std::tuple{0.0, 0.0}, {4.0, 0.0}, {4.0, 1.0}, {0.0, 1.0}}));
}

TEST_F(LeftOfObstaclePredicateTest, Positive) {
    auto restricted = positive_pred->restrict_reach_node(TestEnvironments::time_step, reach_node_one->clone(), {},
                                                         test_envs.world_one, test_envs.ccs_one);
    // Splitting is not necessary for this predicate
    ASSERT_EQ(restricted.size(), 1);
    auto restricted_node = restricted.at(0);
    // expect lat min set to obstacle left inflated by ego width
    double expected_lat_min = 2.0 + (2.0 / 2.0) + ego_width / 2.0;
    EXPECT_NEAR(restricted_node->p_lat_min(), expected_lat_min, tolerance);
    // expect other bounds to be unchanged
    EXPECT_EQ(restricted_node->p_lat_max(), reach_node_one->p_lat_max());
    EXPECT_EQ(restricted_node->v_lat_min(), reach_node_one->v_lat_min());
    EXPECT_EQ(restricted_node->v_lat_max(), reach_node_one->v_lat_max());
    EXPECT_EQ(restricted_node->box_lon(), reach_node_one->box_lon());
}

TEST_F(LeftOfObstaclePredicateTest, Negative) {
    auto restricted = negative_pred->restrict_reach_node(TestEnvironments::time_step, reach_node_one->clone(), {},
                                                         test_envs.world_one, test_envs.ccs_one);
    // Splitting is not necessary for this predicate
    ASSERT_EQ(restricted.size(), 1);
    auto restricted_node = restricted.at(0);
    // expect lat max set to obstacle left inflated by ego width
    double expected_lat_max = 2.0 + (2.0 / 2.0) + ego_width / 2.0;
    EXPECT_NEAR(restricted_node->p_lat_max(), expected_lat_max, tolerance);
    // expect other bounds to be unchanged
    EXPECT_EQ(restricted_node->p_lat_min(), reach_node_one->p_lat_min());
    EXPECT_EQ(restricted_node->v_lat_min(), reach_node_one->v_lat_min());
    EXPECT_EQ(restricted_node->v_lat_max(), reach_node_one->v_lat_max());
    EXPECT_EQ(restricted_node->box_lon(), reach_node_one->box_lon());
}
