#include "test_left_of_obstacle_predicate.hpp"

void LeftOfObstaclePredicateTest::SetUp() {
    test_envs.set_up_environments();
    positive_pred = std::make_unique<semantic_reach::LeftOfObstaclePredicate>(
        false, TestEnvironments::id_obstacle_one, ego_length);
    negative_pred = std::make_unique<semantic_reach::LeftOfObstaclePredicate>(
        true, TestEnvironments::id_obstacle_one, ego_length);

    reach_node_one = std::make_shared<reach::ReachNode>(
        TestEnvironments::time_step,
        std::make_shared<reach::ReachPolygon>(std::vector{std::tuple{0.0, 0.0}, {50.0, 0.0}, {50.0, 5.0}, {0.0, 5.0}}),
        std::make_shared<reach::ReachPolygon>(std::vector{std::tuple{0.0, 0.0}, {4.0, 0.0}, {4.0, 1.0}, {0.0, 1.0}}));
}

TEST_F(LeftOfObstaclePredicateTest, Positive) {
    auto restricted = positive_pred->restrict_reach_node(TestEnvironments::time_step, reach_node_one, {},
                                                         test_envs.world_one, test_envs.ccs_one);
    // Splitting is not necessary for this predicate
    ASSERT_EQ(restricted.size(), 1);
    auto restricted_node = restricted.at(0);
    // expect lon min set to obstacle front inflated by ego length
    double expected_lon_min = 20.0 + (5.0 / 2.0) + ego_length / 2.0;
    EXPECT_NEAR(restricted_node->p_lon_min(), expected_lon_min, tolerance);
    // expect other bounds to be unchanged
    EXPECT_EQ(restricted_node->p_lon_max(), reach_node_one->p_lon_max());
    EXPECT_EQ(restricted_node->v_lon_min(), reach_node_one->v_lon_min());
    EXPECT_EQ(restricted_node->v_lon_max(), reach_node_one->v_lon_max());
    EXPECT_EQ(restricted_node->box_lat(), reach_node_one->box_lat());
}

TEST_F(LeftOfObstaclePredicateTest, Negative) {
    auto restricted = negative_pred->restrict_reach_node(TestEnvironments::time_step, reach_node_one, {},
                                                         test_envs.world_one, test_envs.ccs_one);
    // Splitting is not necessary for this predicate
    ASSERT_EQ(restricted.size(), 1);
    auto restricted_node = restricted.at(0);
    // expect lon max set to obstacle front inflated by ego length
    double expected_lon_max = 20.0 + (5.0 / 2.0) + ego_length / 2.0;
    EXPECT_NEAR(restricted_node->p_lon_max(), expected_lon_max, tolerance);
    // expect other bounds to be unchanged
    EXPECT_EQ(restricted_node->p_lon_min(), reach_node_one->p_lon_min());
    EXPECT_EQ(restricted_node->v_lon_min(), reach_node_one->v_lon_min());
    EXPECT_EQ(restricted_node->v_lon_max(), reach_node_one->v_lon_max());
    EXPECT_EQ(restricted_node->box_lat(), reach_node_one->box_lat());
}
