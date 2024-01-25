#include "reach_semantic/data_structure/reach/predicates/position/keeps_safe_distance_prec_predicate.hpp"
#include <commonroad_cpp/obstacle/obstacle.h>
#include <spdlog/spdlog.h>


using namespace semantic_reach;
using geometry::CurvilinearCoordinateSystem;

KeepSafeDistancePrecPredicate :: KeepSafeDistancePrecPredicate(bool negated, size_t obstacle_id, double ego_length)
    : CppPredicate(negated, false), obstacle_id(obstacle_id), ego_length(ego_length) {}

std::vector<reach::ReachNodePtr> KeepSafeDistancePrecPredicate::_restrict_reach_node_mandatory(
    int step, const reach::ReachNodePtr &reach_node, const std::shared_ptr<World> &world,
    const std::shared_ptr<CurvilinearCoordinateSystem> &ego_ccs) const {

    auto min_ego_velocity = reach_node->v_lon_min();
    auto ego_velocity_range = reach_node->v_lon_max() - reach_node->v_lon_min();
    auto ego_velocity_increment = ego_velocity_range / static_cast<double>(NUM_SUPPORT_POINTS - 1);
    for (int i = 0; i < NUM_SUPPORT_POINTS; i++) {
        auto ego_velocity_support = min_ego_velocity + i * ego_velocity_increment;
        //        1. version
        //        double a = -1 * _determine_slope(step, reach_node, world, ego_ccs, ego_velocity_support);
        //        double b = 1;
        //        double c = _determine_constant_b(step, reach_node, world, ego_ccs, ego_velocity_support);

        //        2. version x<=slope(ego_velocity_support) * y + safe_position(ego_velocity_support);
        //        reformulated: x - slope(ego_velocity_support) * y <= safe_position(ego_velocity_support)
        // TODO: Remove unnecessary parameters and move configuration (e.g., decelerations) here (but outside the loop)
        auto slope = _determine_slope(step, reach_node, world, ego_ccs, ego_velocity_support);
        auto safe_pos = _determine_safe_position(step, reach_node, world, ego_ccs, ego_velocity_support);
        // TODO: Put this into a function returning std::tuple<double, double, double>
        double a = 1;
        double b = -slope;
        double c = safe_pos - slope * ego_velocity_support;

        reach_node->polygon_lon->intersect_halfspace(a, b, c);
    }

    return {reach_node};
}

std::vector<reach::ReachNodePtr> KeepSafeDistancePrecPredicate::_restrict_reach_node_forbidden(
    int step, const reach::ReachNodePtr &reach_node, const std::shared_ptr<World> &world,
    const std::shared_ptr<CurvilinearCoordinateSystem> &ego_ccs) const {

    double safe_position_for_given_velocity[20];
    reach::ReachNodePtr node[20];
    auto obstacle = world->findObstacle(obstacle_id);
    // TODO: check for error
    std::shared_ptr<State> obstacle_state = obstacle->getStateByTimeStep(step);
    try {
        obstacle_state = obstacle->getStateByTimeStep(step);
    } catch (std::logic_error &e) {

    }
    double vehicle_speed = obstacle_state -> getVelocity();
    double vehicle_deceleration = -10.5;
    double ego_speed = 36.66; //for now
    double ego_reaction_time = 0.3;
    double ego_deceleration = -10.0;
    double ego_speed_change = reach_node->v_lon_max() - reach_node->v_lon_min();
    double start_velocity = reach_node->v_lon_min();
    for(int i = 0; i<20; i++) {
        node[i] = reach_node->clone(); // Clone the reach node, because we need a union
        safe_position_for_given_velocity[i] = _determine_safe_position(step, reach_node, world, ego_ccs, start_velocity + ego_speed_change*i/19);
        node[i]->polygon_lon->intersect_halfspace( (ego_speed /abs(ego_deceleration) - ego_reaction_time), -1,  (vehicle_speed * vehicle_speed) / (-2 * abs(vehicle_deceleration)) + (3*(ego_speed * ego_speed) / (2 * abs(ego_deceleration) - 2 * ego_speed * ego_reaction_time)));
    }
    //    std::vector<reach::ReachNodePtr> v{};
    //    v.reserve(20);
    //    for ...
    //    v.push_back();
    std::vector<reach::ReachNodePtr> v(node, node + sizeof node / sizeof node[0]);
    return {v};

}


// It returns max lon position, that is still safe for ego
double KeepSafeDistancePrecPredicate::_determine_safe_position(
    int step, const reach::ReachNodePtr &reach_node, const std::shared_ptr<World> &world,
    const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ego_ccs, double ego_speed) const {

    auto obstacle = world->findObstacle(obstacle_id);
    // TODO: check for logic error and throw error
    std::shared_ptr<State> obstacle_state = obstacle->getStateByTimeStep(step);
    try {
        obstacle_state = obstacle->getStateByTimeStep(step);
    } catch (std::logic_error &e) {
        // TODO: Handle this error, i.e., log a warning message and keep the entire reach node
        // reason: if we don't have a prediction for a given time step, we assume that the other vehicle is far away
    }

    double vehicle_speed = obstacle_state -> getVelocity(); //speed of other vehicle
    double vehicle_deceleration = -10.5;                    //deceleration of other vehicle
    double ego_reaction_time = 0.3;
    double ego_deceleration = -10.0;

    double safe_dist = (vehicle_speed * vehicle_speed) / (-2 * abs(vehicle_deceleration))
                       - (ego_speed * ego_speed) / (-2 * abs(ego_deceleration))
                       + ego_speed * ego_reaction_time;

    return obstacle->rearS(step, ego_ccs) - safe_dist - ego_length / 2.0;
}

double KeepSafeDistancePrecPredicate::_determine_slope(
    int step, const reach::ReachNodePtr &reach_node, const std::shared_ptr<World> &world,
    const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ego_ccs, double ego_speed) const{
    double ego_deceleration = -10.0;
    double ego_reaction_time = 0.3;
    double v_prim = ego_speed/abs(ego_deceleration) + ego_reaction_time;

    // negate the slope of the safe distance function to get the slope of the safe position function (safe distance is
    // negated there)
    return -v_prim;
}
// slope line equation: y = f'(x0) * x + b
double KeepSafeDistancePrecPredicate::_determine_constant_b(
    int step, const reach::ReachNodePtr &reach_node, const std::shared_ptr<World> &world,
    const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ego_ccs, double ego_speed) const{
    double s = _determine_safe_position(step, reach_node, world, ego_ccs, ego_speed);
    double v_prim = _determine_slope(step, reach_node, world, ego_ccs, ego_speed);
    double b = s - v_prim * ego_speed;
    return b;
}




