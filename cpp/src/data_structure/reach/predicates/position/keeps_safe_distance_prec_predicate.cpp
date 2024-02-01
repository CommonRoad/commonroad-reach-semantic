#include "reach_semantic/data_structure/reach/predicates/position/keeps_safe_distance_prec_predicate.hpp"
#include <commonroad_cpp/obstacle/obstacle.h>
#include <spdlog/spdlog.h>

using namespace semantic_reach;
using geometry::CurvilinearCoordinateSystem;

KeepSafeDistancePrecPredicate ::KeepSafeDistancePrecPredicate(bool negated, size_t obstacle_id, double ego_length)
    : CppPredicate(negated, false), obstacle_id(obstacle_id), ego_length(ego_length) {}

std::vector<reach::ReachNodePtr> KeepSafeDistancePrecPredicate::_restrict_reach_node_mandatory(
    int step, const reach::ReachNodePtr &reach_node, const std::shared_ptr<World> &world,
    const std::shared_ptr<CurvilinearCoordinateSystem> &ego_ccs) const {

    auto min_ego_velocity = reach_node->v_lon_min();
    auto ego_velocity_range = reach_node->v_lon_max() - reach_node->v_lon_min();
    auto ego_velocity_increment = ego_velocity_range / static_cast<double>(NUM_SUPPORT_POINTS - 1);
    for (int i = 0; i < NUM_SUPPORT_POINTS; i++) {
        auto ego_velocity_support = min_ego_velocity + i * ego_velocity_increment;
        auto safe_position_opt = _determine_safe_position(step, reach_node, world, ego_ccs, ego_velocity_support);
        if (!safe_position_opt.has_value()) {
            spdlog::warn("No prediction for obstacle {} at step {}, cannot restrict reach node.", obstacle_id, step);
            return {reach_node};
        }
        auto slope = _determine_slope(ego_velocity_support);

        //        x<=slope(ego_velocity_support) * y + safe_position(ego_velocity_support);
        //        reformulated: x - slope(ego_velocity_support) * y <= safe_position(ego_velocity_support)
        auto [a, b, c] = _compute_parameters_of_halfspace(ego_velocity_support, slope, safe_position_opt.value());

        reach_node->polygon_lon->intersect_halfspace(a, b, c);
    }
    return {reach_node};
}

std::vector<reach::ReachNodePtr> KeepSafeDistancePrecPredicate::_restrict_reach_node_forbidden(
    int step, const reach::ReachNodePtr &reach_node, const std::shared_ptr<World> &world,
    const std::shared_ptr<CurvilinearCoordinateSystem> &ego_ccs) const {
    std::vector<reach::ReachNodePtr> v{};
    v.reserve(NUM_SUPPORT_POINTS);

    auto min_ego_velocity = reach_node->v_lon_min();
    auto ego_velocity_range = reach_node->v_lon_max() - reach_node->v_lon_min();
    auto ego_velocity_increment = ego_velocity_range / static_cast<double>(NUM_SUPPORT_POINTS - 1);
    for (int i = 0; i < NUM_SUPPORT_POINTS; i++) {
        auto node = reach_node->clone();
        auto ego_velocity_support = min_ego_velocity + i * ego_velocity_increment;
        auto safe_position_opt = _determine_safe_position(step, reach_node, world, ego_ccs, ego_velocity_support);
        if (!safe_position_opt.has_value()) {
            spdlog::warn("No prediction for obstacle {} at step {}, cannot restrict reach node.", obstacle_id, step);
            continue;
        }
        auto slope = _determine_slope(ego_velocity_support);
        //        x<=slope(ego_velocity_support) * y + safe_position(ego_velocity_support);
        //        reformulated: x - slope(ego_velocity_support) * y <= safe_position(ego_velocity_support)
        auto [a, b, c] = _compute_parameters_of_halfspace(ego_velocity_support, slope, safe_position_opt.value());

        node->polygon_lon->intersect_halfspace(-a, -b, -c);
        v.push_back(node);
    }
    return v;
}

// It returns max lon position, that is still safe for ego
std::optional<double> KeepSafeDistancePrecPredicate::_determine_safe_position(
    int step, const reach::ReachNodePtr &reach_node, const std::shared_ptr<World> &world,
    const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ego_ccs, double ego_speed) const {

    auto obstacle = world->findObstacle(obstacle_id);
    std::shared_ptr<State> obstacle_state;
    try {
        obstacle_state = obstacle->getStateByTimeStep(step);
    } catch (std::logic_error &e) {
        return std::nullopt;
        // reason: if we don't have a prediction for a given time step, we assume that the other vehicle is far away
    }

    double vehicle_speed = obstacle_state->getVelocity(); // speed of other vehicle

    double safe_dist = (vehicle_speed * vehicle_speed) / (-2 * abs(vehicle_deceleration)) -
                       (ego_speed * ego_speed) / (-2 * abs(ego_deceleration)) + ego_speed * ego_reaction_time;

    return obstacle->rearS(step, ego_ccs) - safe_dist - ego_length / 2.0;
}

double KeepSafeDistancePrecPredicate::_determine_slope(double ego_speed) const {
    double v_prim = ego_speed / abs(ego_deceleration) + ego_reaction_time;
    // negate the slope of the safe distance function to get the slope of the safe position function (safe distance is
    // negated there)
    return -v_prim;
}

std::tuple<double, double, double>
KeepSafeDistancePrecPredicate::_compute_parameters_of_halfspace(double ego_velocity_support, double slope,
                                                                double safe_pos) {
    std::tuple<double, double, double> parameters;
    parameters = make_tuple(1, -slope, safe_pos - slope * ego_velocity_support);
    return parameters;
}
