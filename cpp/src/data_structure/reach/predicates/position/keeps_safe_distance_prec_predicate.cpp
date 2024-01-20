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

    double safe_position_for_given_velocity[20];
    auto obstacle = world->findObstacle(obstacle_id);
    // TODO: check for error
    std::shared_ptr<State> obstacle_state = obstacle->getStateByTimeStep(step);
//    try {
//        obstacle_state = obstacle->getStateByTimeStep(step);
//    } catch (std::logic_error &e) {
//
//    }
    double vehicle_speed = obstacle_state->getVelocity(); //vehicle.v_lon_ref(step)
    double vehicle_deceleration = -10.5;                  // semantic_model.config.vehicle.other.a_lon_min
    double ego_speed;
    double ego_reaction_time = 0.3; //semantic_model.config.vehicle.ego.t_react
    double ego_deceleration = -10.0; // semantic_model.config.vehicle.ego.a_lon_min

    for(int i = 0; i<20; i++) {
        double ego_speed_change = reach_node->v_lon_max() - reach_node->v_lon_min();
        double start_velocity = reach_node->v_lon_min();
        ego_speed = start_velocity + ego_speed_change * i / 19;

        safe_position_for_given_velocity[i] = _determine_safe_position(step, reach_node, world, ego_ccs, start_velocity + ego_speed_change*i/19);
        reach_node->polygon_lon->intersect_halfspace( -(ego_speed /abs(ego_deceleration) + ego_reaction_time), 1, - (vehicle_speed * vehicle_speed) / (-2 * abs(vehicle_deceleration)) - (3*(ego_speed * ego_speed) / (2 * abs(ego_deceleration) +2 * ego_speed * ego_reaction_time)));
    }
        // f(es) = (vs^2) / (-2 * abs(vd)) - (es^2) / (-2 * abs(ed)) + es * er;
        // f'(es) =  es /abs(ed) + er;
        //  ax + by <= c
        //y <= f'(es)*(x-es) + f(es)
        //y <= (es /abs(ed) + er)*(x-es) + (vs^2) / (-2 * abs(vd)) - (es^2) / (-2 * abs(ed)) + es * er
        //y <= (es /abs(ed) + er)*x-(es*es /abs(ed) + er*es + (vs^2) / (-2 * abs(vd)) + (es^2) / (2 * abs(ed)) + es * er

        //y <= (es /abs(ed) + er)*x - (vs^2) / (-2 * abs(vd)) - 3*(es^2) / (2 * abs(ed) +2* es * er
        //a = -(es /abs(ed) + er)
        //b = 1
        //c = - (vs^2) / (-2 * abs(vd)) - 3*(es^2) / (2 * abs(ed) +2* es * er
    //i-th point: x = max_velocity*i/19, y = safe_position_for_given_velocity[i].second
    //reach_node ->intersect_in_position_domain()
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
//    try {
//        obstacle_state = obstacle->getStateByTimeStep(step);
//    } catch (std::logic_error &e) {
//
//    }
    double vehicle_speed = obstacle_state -> getVelocity();
    double vehicle_deceleration = -10.5;
    double ego_speed = 36.66; //for now
    double ego_reaction_time = 0.3;
    double ego_deceleration = -10.0;
    // f(es) = (vs^2) / (-2 * abs(vd)) - (es^2) / (-2 * abs(ed)) + es * er;
    // f'(es) =  es /abs(ed) + er;
    //  ax + by <= c
    // (es /abs(ed) + er)*x - (vs^2) / (-2 * abs(vd)) - 3*(es^2) / (2 * abs(ed) +2* es * er <= y

    // a = (es /abs(ed) + er)
    //b = -1
    //c = (vs^2) / (-2 * abs(vd)) + 3*(es^2) / (2 * abs(ed) - 2* es * er
    for(int i = 0; i<20; i++) {
        node[i] = reach_node;
        double ego_speed_change = reach_node->v_lon_max() - reach_node->v_lon_min();
        double start_velocity = reach_node->v_lon_min();


        safe_position_for_given_velocity[i] = _determine_safe_position(step, reach_node, world, ego_ccs, start_velocity + ego_speed_change*i/19);
        node[i]->polygon_lon->intersect_halfspace( (ego_speed /abs(ego_deceleration) - ego_reaction_time), -1,  (vehicle_speed * vehicle_speed) / (-2 * abs(vehicle_deceleration)) + (3*(ego_speed * ego_speed) / (2 * abs(ego_deceleration) - 2 * ego_speed * ego_reaction_time)));
    }
    std::vector<reach::ReachNodePtr> v(node, node + sizeof node / sizeof node[0]);
    return {v};

}


// It returns max lon position, that is still safe for ego
double KeepSafeDistancePrecPredicate::_determine_safe_position(
    int step, const reach::ReachNodePtr &reach_node, const std::shared_ptr<World> &world,
    const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ego_ccs, double ego_speed) const {

    auto obstacle = world->findObstacle(obstacle_id);
    // TODO: check for error
    std::shared_ptr<State> obstacle_state = obstacle->getStateByTimeStep(step);
//    try {
//        obstacle_state = obstacle->getStateByTimeStep(step);
//    } catch (std::logic_error &e) {
//
//    }

    double vehicle_speed = obstacle_state -> getVelocity(); //speed of other vehicle
    double vehicle_deceleration = -10.5;                    //deceleration of other vehicle
    double ego_reaction_time = 0.3;
    double ego_deceleration = -10.0;

    double safe_dist = (vehicle_speed * vehicle_speed) / (-2 * abs(vehicle_deceleration))
                       - (ego_speed * ego_speed) / (-2 * abs(ego_deceleration))
                       + ego_speed * ego_reaction_time;



    return obstacle -> rearS(step, ego_ccs) - safe_dist - ego_length / 2.0;

}

double KeepSafeDistancePrecPredicate::_determine_shape(
    int step, const reach::ReachNodePtr &reach_node, const std::shared_ptr<World> &world,
    const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ego_ccs, double ego_speed) const{

    return 1.0;
}




