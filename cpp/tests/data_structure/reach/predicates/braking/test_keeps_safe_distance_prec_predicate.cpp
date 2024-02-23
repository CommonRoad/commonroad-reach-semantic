#include "test_keeps_safe_distance_prec_predicate.hpp"

void KeepsSafeDistancePrecPredicateTest::SetUp() {
    test_envs.set_up_environments();
    positive_pred = std::make_unique<semantic_reach::KeepsSafeDistancePrecPredicate>(
        false, TestEnvironments::id_obstacle_one, ego_length, ego_reaction_time, ego_deceleration, other_deceleration);
    negative_pred = std::make_unique<semantic_reach::KeepsSafeDistancePrecPredicate>(
        true, TestEnvironments::id_obstacle_one, ego_length, ego_reaction_time, ego_deceleration, other_deceleration);

    reach_node_one = std::make_shared<reach::ReachNode>(
        TestEnvironments::time_step,
        std::make_shared<reach::ReachPolygon>(std::vector{std::tuple{0.0, 0.0}, {50.0, 0.0}, {50.0, 5.0}, {0.0, 5.0}}),
        std::make_shared<reach::ReachPolygon>(std::vector{std::tuple{0.0, 0.0}, {4.0, 0.0}, {4.0, 1.0}, {0.0, 1.0}}));
}

TEST_F(KeepsSafeDistancePrecPredicateTest, Positive) {
    auto restricted = positive_pred->restrict_reach_node(TestEnvironments::time_step, reach_node_one->clone(), {},
                                                         test_envs.world_one, test_envs.ccs_one);
    // Splitting is not necessary for this predicate
    ASSERT_EQ(restricted.size(), 1);
    auto restricted_node = restricted.at(0);

    // Sample velocities between 0 and 5
    for (int i = 0; i < 20; i++) {
        auto velocity = i / 4.0;
        auto safe_position = safe_position_world_one(velocity);
        auto position_in =
            fmax(fmin(safe_position - tolerance, reach_node_one->p_lon_max()), reach_node_one->p_lon_min());
        EXPECT_TRUE(contains_point(*restricted_node->polygon_lon, position_in, velocity));
        auto position_out = safe_position + overapprox_tolerance; // More tolerance here, because we overapproximate
        EXPECT_FALSE(contains_point(*restricted_node->polygon_lon, position_out, velocity));
    }
}

TEST_F(KeepsSafeDistancePrecPredicateTest, Negative) {
    auto restricted = negative_pred->restrict_reach_node(TestEnvironments::time_step, reach_node_one->clone(), {},
                                                         test_envs.world_one, test_envs.ccs_one);

    // Sample velocities between 0 and 5
    for (int i = 0; i < 20; i++) {
        auto velocity = i / 4.0;
        auto safe_position = safe_position_world_one(velocity);
        auto position_in =
            fmax(fmin(safe_position + tolerance, reach_node_one->p_lon_max()), reach_node_one->p_lon_min());
        auto any_contains = std::any_of(restricted.begin(), restricted.end(), [&](const auto &node) {
            return contains_point(*node->polygon_lon, position_in, velocity);
        });
        EXPECT_TRUE(any_contains);
        auto position_out = safe_position - overapprox_tolerance; // More tolerance here, because we overapproximate
        auto none_contains = std::none_of(restricted.begin(), restricted.end(), [&](const auto &node) {
            return contains_point(*node->polygon_lon, position_out, velocity);
        });
        EXPECT_TRUE(none_contains);
    }
}

double KeepsSafeDistancePrecPredicateTest::safe_position_world_one(double ego_velocity) {
    // others velocity is 5 --> so its square is 25
    auto safe_dist = 25.0 / (-2 * abs(other_deceleration)) - pow(ego_velocity, 2) / (-2 * abs(ego_deceleration)) +
                     ego_reaction_time * ego_velocity;

    // others lon position is 20 and its length is 5
    return 20.0 - 5.0 / 2.0 - safe_dist - ego_length / 2.0;
}

bool KeepsSafeDistancePrecPredicateTest::contains_point(const reach::ReachPolygon &polygon, double position,
                                                        double velocity) {
    auto const &vertices = polygon.vertices();

    // Ray casting algorithm to determine if point is inside polygon
    int intersections = 0;
    for (int i = 0; i < vertices.size(); i++) {
        auto const &cur = vertices[i];
        // If the point matches a vertex, then it is inside
        if (abs(cur.x - position) < 2 * tolerance && abs(cur.y - velocity) < 2 * tolerance) {
            return true;
        }
        auto const &next = vertices[(i + 1) % vertices.size()];
        // If point is not between the x coordinates of the vertices, then it cannot intersect the edge
        if (position < fmin(cur.x, next.x) || position > fmax(cur.x, next.x)) {
            continue;
        }
        // Cast ray from point to top and check if it intersects the edge between cur and next
        auto m = (next.y - cur.y) / (next.x - cur.x);
        auto t = cur.y - m * cur.x;
        if (m * position + t >= velocity) {
            intersections++;
        }
    }
    // If we intersect an edge an odd number of times, the point is inside the polygon
    return intersections % 2 == 1;
}
