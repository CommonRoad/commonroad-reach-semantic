#include "reach_semantic/data_structure/reach/predicates/position/in_front_of_obstacle_predicate.hpp"

#include <commonroad_cpp/obstacle/obstacle.h>
#include <spdlog/spdlog.h>
#include <regex>

using namespace semantic_reach;
using geometry::CurvilinearCoordinateSystem;

InFrontOfObstaclePredicate::InFrontOfObstaclePredicate(std::shared_ptr<PredicateConfiguration> config, bool negated,
                                                       int obstacle_id) : CppPredicate(std::move(config), negated,
                                                                                       false),
                                                                          obstacle_id(obstacle_id) {}

std::vector<reach::ReachNodePtr>
InFrontOfObstaclePredicate::_restrict_reach_node_mandatory(int step, const reach::ReachNodePtr &reach_node,
                                                           const std::shared_ptr<World> &world,
                                                           const std::shared_ptr<CurvilinearCoordinateSystem> &ego_ccs) const {
    auto obstacle_front = _get_obstacle_front(step, world, ego_ccs);
    if (!obstacle_front.has_value()) {
        spdlog::warn("No prediction for obstacle {} at step {}, cannot restrict reach node.", obstacle_id, step);
        return {reach_node};
    }

    reach_node->intersect_in_position_domain(obstacle_front.value());

    return {reach_node};
}

std::vector<reach::ReachNodePtr>
InFrontOfObstaclePredicate::_restrict_reach_node_forbidden(int step, const reach::ReachNodePtr &reach_node,
                                                           const std::shared_ptr<World> &world,
                                                           const std::shared_ptr<CurvilinearCoordinateSystem> &ego_ccs) const {
    auto obstacle_front = _get_obstacle_front(step, world, ego_ccs);
    if (!obstacle_front.has_value()) {
        spdlog::warn("No prediction for obstacle {} at step {}, cannot restrict reach node.", obstacle_id, step);
        return {reach_node};
    }

    reach_node->intersect_in_position_domain(-std::numeric_limits<double>::infinity(),
                                             -std::numeric_limits<double>::infinity(), obstacle_front.value());

    return {reach_node};
}

std::optional<double> InFrontOfObstaclePredicate::_get_obstacle_front(int step, const std::shared_ptr<World> &world,
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
    return obstacle->frontS(step, ego_ccs) + config->ego_length / 2.0;
}

std::optional<std::unique_ptr<InFrontOfObstaclePredicate>>
InFrontOfObstaclePredicate::try_from_proposition(const std::string &proposition,
                                                 const std::shared_ptr<PredicateConfiguration> &config, bool negated) {
    std::smatch match;
    if (std::regex_match(proposition, match, std::regex(R"(InFrontOf_V(\d+))"))) {
        int obstacle_id{std::stoi(match[1])};
        return std::make_unique<InFrontOfObstaclePredicate>(config, negated, obstacle_id);
    }
    return std::nullopt;
}
