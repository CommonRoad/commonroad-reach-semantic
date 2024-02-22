#include "reach_semantic/data_structure/reach/predicates/position/left_of_obstacle_predicate.hpp"

#include <commonroad_cpp/obstacle/obstacle.h>
#include <spdlog/spdlog.h>

using namespace semantic_reach;
using geometry::CurvilinearCoordinateSystem;

LeftOfObstaclePredicate::LeftOfObstaclePredicate(bool negated, size_t obstacle_id, double ego_length)
    : CppPredicate(negated, false), obstacle_id(obstacle_id), ego_width(ego_length) {}

std::vector<reach::ReachNodePtr> LeftOfObstaclePredicate::_restrict_reach_node_mandatory(
    int step, const reach::ReachNodePtr &reach_node, const std::shared_ptr<World> &world,
    const std::shared_ptr<CurvilinearCoordinateSystem> &ego_ccs) const {
    auto obstacle_left = _get_obstacle_left(step, world, ego_ccs);
    if (!obstacle_left.has_value()) {
        spdlog::warn("No prediction for obstacle {} at step {}, cannot restrict reach node.", obstacle_id, step);
        return {reach_node};
    }

    reach_node->intersect_in_position_domain(-std::numeric_limits<double>::infinity(), obstacle_left.value());

    return {reach_node};
}

std::vector<reach::ReachNodePtr> LeftOfObstaclePredicate::_restrict_reach_node_forbidden(
    int step, const reach::ReachNodePtr &reach_node, const std::shared_ptr<World> &world,
    const std::shared_ptr<CurvilinearCoordinateSystem> &ego_ccs) const {
    auto obstacle_left = _get_obstacle_left(step, world, ego_ccs);
    if (!obstacle_left.has_value()) {
        spdlog::warn("No prediction for obstacle {} at step {}, cannot restrict reach node.", obstacle_id, step);
        return {reach_node};
    }

    reach_node->intersect_in_position_domain(-std::numeric_limits<double>::infinity(),
                                             -std::numeric_limits<double>::infinity(),
                                             std::numeric_limits<double>::infinity(), obstacle_left.value());

    return {reach_node};
}

std::optional<double>
LeftOfObstaclePredicate::_get_obstacle_left(int step, const std::shared_ptr<World> &world,
                                            const std::shared_ptr<CurvilinearCoordinateSystem> &ego_ccs) const {
    auto obstacle = world->findObstacle(obstacle_id);
    std::shared_ptr<State> obstacle_state;
    try {
        obstacle_state = obstacle->getStateByTimeStep(step);
    } catch (std::logic_error &e) {
        return std::nullopt;
    }
    if (!ego_ccs->cartesianPointInProjectionDomain(obstacle_state->getXPosition(), obstacle_state->getYPosition())) {
        return std::nullopt;
    }

    // inflate by half the length of the ego vehicle, as we use the center for reference
    return obstacle->leftD(step, ego_ccs) + ego_width / 2.0;
}
