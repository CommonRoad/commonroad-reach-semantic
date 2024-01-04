#include "reach_semantic/data_structure/reach/predicates/predicate_config.hpp"

using namespace semantic_reach;

PredicateConfiguration::PredicateConfiguration(const SemanticConfiguration &config) {
    traffic_rule_params = PredicateParameters{};
    auto traffic_rule_config = config.config_traffic_rule;
    traffic_rule_params.updateParam("standstillError", traffic_rule_config.backward_driving_v_err);
    traffic_rule_params.updateParam("aBrakingIntersection", traffic_rule_config.acceleration_braking_hard);
    traffic_rule_params.updateParam("dBrakingIntersection", traffic_rule_config.distance_braking);
    traffic_rule_params.checkParameterValidity();

    ego_length = config.config_vehicle.ego.length;
    ego_width = config.config_vehicle.ego.width;
}
