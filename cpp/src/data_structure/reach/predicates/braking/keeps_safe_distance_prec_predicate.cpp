#include "reach_semantic/data_structure/reach/predicates/braking/keeps_safe_distance_prec_predicate.hpp"
#include "commonroad_cpp/obstacle/obstacle.h"
#include <spdlog/spdlog.h>

using namespace semantic_reach;
using geometry::CurvilinearCoordinateSystem;

KeepsSafeDistancePrecPredicate ::KeepsSafeDistancePrecPredicate(bool negated, size_t obstacle_id, double ego_length,
                                                                double ego_reaction_time, double ego_deceleration,
                                                                double vehicle_deceleration)
    : CppPredicate(negated, false), obstacle_id(obstacle_id), ego_length(ego_length),
      ego_reaction_time(ego_reaction_time), ego_deceleration(ego_deceleration),
      other_deceleration(vehicle_deceleration) {}

std::vector<reach::ReachNodePtr> KeepsSafeDistancePrecPredicate::_restrict_reach_node_mandatory(
    int step, const reach::ReachNodePtr &reach_node, const std::shared_ptr<World> &world,
    const std::shared_ptr<CurvilinearCoordinateSystem> &ego_ccs) const {

    auto min_ego_velocity = reach_node->v_lon_min();
    auto ego_velocity_range = reach_node->v_lon_max() - reach_node->v_lon_min();
    auto ego_velocity_increment = ego_velocity_range / static_cast<double>(NUM_SUPPORT_POINTS - 1);
    for (int i = 0; i < NUM_SUPPORT_POINTS; i++) {
        auto ego_velocity_support = min_ego_velocity + i * ego_velocity_increment;

        auto pos_v_opt = _get_other_position_and_velocity(step, world, ego_ccs);
        if (!pos_v_opt.has_value()) {
            spdlog::warn("No prediction for obstacle {} at step {}, assuming the vehicle is far away.", obstacle_id,
                         step);
            return {reach_node};
        }
        auto [other_position, other_velocity] = pos_v_opt.value();

        auto safe_position = _determine_safe_position(ego_velocity_support, other_position, other_velocity);
        auto slope = _determine_slope(ego_velocity_support);

        auto [a, b, c] = _compute_halfspace_coefficients(ego_velocity_support, slope, safe_position);
        reach_node->polygon_lon->intersect_halfspace(a, b, c);
    }
    return {reach_node};
}

std::vector<reach::ReachNodePtr> KeepsSafeDistancePrecPredicate::_restrict_reach_node_forbidden(
    int step, const reach::ReachNodePtr &reach_node, const std::shared_ptr<World> &world,
    const std::shared_ptr<CurvilinearCoordinateSystem> &ego_ccs) const {
    std::vector<reach::ReachNodePtr> v{};
    v.reserve(NUM_SUPPORT_POINTS);

    auto min_ego_velocity = reach_node->v_lon_min();
    auto ego_velocity_range = reach_node->v_lon_max() - reach_node->v_lon_min();
    auto ego_velocity_increment = ego_velocity_range / static_cast<double>(NUM_SUPPORT_POINTS - 1);
    for (int i = 0; i < NUM_SUPPORT_POINTS - 1; i++) {
        auto ego_velocity_support = min_ego_velocity + i * ego_velocity_increment;
        auto next_support = min_ego_velocity + (i + 1) * ego_velocity_increment;

        auto pos_v_opt = _get_other_position_and_velocity(step, world, ego_ccs);
        if (!pos_v_opt.has_value()) {
            spdlog::warn("No prediction for obstacle {} at step {}, assuming the vehicle is far away.", obstacle_id,
                         step);
            return {};
        }
        auto [other_position, other_velocity] = pos_v_opt.value();

        auto safe_position = _determine_safe_position(ego_velocity_support, other_position, other_velocity);
        auto next_safe_position = _determine_safe_position(next_support, other_position, other_velocity);
        auto secant_slope =
            _determine_secant_slope(ego_velocity_support, next_support, safe_position, next_safe_position);

        auto [a, b, c] = _compute_halfspace_coefficients(ego_velocity_support, secant_slope, safe_position);
        auto node = reach_node->clone();
        // negate the halfspace parameters to get the forbidden region
        node->polygon_lon->intersect_halfspace(-a, -b, -c);
        v.push_back(node);
    }
    return v;
}

double KeepsSafeDistancePrecPredicate::_determine_safe_position(double ego_velocity, double other_position,
                                                                double other_velocity) const {
    double safe_dist = (other_velocity * other_velocity) / (-2 * abs(other_deceleration)) -
                       (ego_velocity * ego_velocity) / (-2 * abs(ego_deceleration)) + ego_velocity * ego_reaction_time;

    return other_position - safe_dist - ego_length / 2.0;
}

double KeepsSafeDistancePrecPredicate::_determine_slope(double ego_velocity) const {
    double v_prime = ego_velocity / abs(ego_deceleration) + ego_reaction_time;
    // negate the slope of the safe distance function to get the slope of the safe position function (safe distance is
    // negated there)
    return -v_prime;
}

double KeepsSafeDistancePrecPredicate::_determine_secant_slope(double lower_support, double upper_support,
                                                               double lower_value, double upper_value) {
    return (upper_value - lower_value) / (upper_support - lower_support);
}

std::optional<std::pair<double, double>> KeepsSafeDistancePrecPredicate::_get_other_position_and_velocity(
    int step, const std::shared_ptr<World> &world,
    const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ego_ccs) const {
    auto obstacle = world->findObstacle(obstacle_id);
    std::shared_ptr<State> obstacle_state;
    try {
        obstacle_state = obstacle->getStateByTimeStep(step);
    } catch (std::logic_error &e) {
        return std::nullopt;
    }
    return std::make_pair(obstacle->rearS(step, ego_ccs), obstacle_state->getVelocity());
}

std::tuple<double, double, double>
KeepsSafeDistancePrecPredicate::_compute_halfspace_coefficients(double ego_velocity_support, double slope,
                                                                double safe_pos) {
    // position <= slope * velocity + safe_pos - slope * ego_velocity_support;
    // in the form a * position + b * velocity <= c:
    // 1 * position + (-slope) * velocity <= safe_pos - slope * ego_velocity_support
    return {1, -slope, safe_pos - slope * ego_velocity_support};
}
