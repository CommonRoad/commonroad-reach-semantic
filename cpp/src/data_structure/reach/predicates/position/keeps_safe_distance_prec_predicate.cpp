#include "reach_semantic/data_structure/reach/predicates/position/keeps_safe_distance_prec_predicate.hpp"
#include <commonroad_cpp/obstacle/obstacle.h>
#include <spdlog/spdlog.h>


using namespace semantic_reach;
using geometry::CurvilinearCoordinateSystem;

KeepSafeDistancePrecPredicate :: KeepSafeDistancePrecPredicate(bool negated, size_t obstacle_id, double ego_length)
    : CppPredicate(negated, false), obstacle_id(obstacle_id), ego_length(ego_length) {};

std::vector<reach::ReachNodePtr> KeepSafeDistancePrecPredicate::_restrict_reach_node_mandatory(
    int step, const reach::ReachNodePtr &reach_node, const std::shared_ptr<World> &world,
    const std::shared_ptr<CurvilinearCoordinateSystem> &ego_ccs) const {

    std::pair <double, double> safe_position_for_given_velocity[20];
    int ego_speed;
    double vehicle_speed = 36.66; //vehicle.v_lon_ref(step)
    double vehicle_deceleration = -6.0; //semantic_model.config.vehicle.other.a_lon_min
    double ego_reaction_time = 0.3; //semantic_model.config.vehicle.ego.t_react
    double ego_deceleration = -6.0; //semantic_model.config.vehicle.ego.a_lon_min
    for(int i = 0; i<20; i++) {
        ego_speed = 36.66;

        safe_position_for_given_velocity[i] = _determine_safe_position(step, reach_node, world, ego_ccs, ego_speed*i/19);
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

}



std::pair<double, double> KeepSafeDistancePrecPredicate::_determine_safe_position(
    int step, const reach::ReachNodePtr &reach_node, const std::shared_ptr<World> &world,
    const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ego_ccs, double ego_speed) const {

    double vehicle_pos = 1;//vehicle.p_lon_min_ref(step, semantic_model.config.vehicle.ego.length / 2)
    double vehicle_speed = 36.66; //vehicle.v_lon_ref(step)
    double vehicle_deceleration = -6.0; //semantic_model.config.vehicle.other.a_lon_min
    double ego_reaction_time = 0.3; //semantic_model.config.vehicle.ego.t_react
    double ego_deceleration = -6.0; //semantic_model.config.vehicle.ego.a_lon_min

    double safe_dist = (vehicle_speed * vehicle_speed) / (-2 * abs(vehicle_deceleration)) - (ego_speed * ego_speed) / (-2 * abs(ego_deceleration)) + ego_speed * ego_reaction_time;


    return std::make_pair(vehicle_pos, safe_dist);

}




