#include "reach_semantic/data_structure/semantic_configuration.hpp"
#include "reachset/utility/shared_using.hpp"

using namespace semantic_reach;

TrafficRuleConfiguration::TrafficRuleConfiguration(const YAML::Node &node) {
    auto node_traffic_rule = node["traffic_rule"];

    distance_braking = node_traffic_rule["distance_braking"].as<double>();
    acceleration_braking_hard = node_traffic_rule["acceleration_braking_hard"].as<double>();
    backward_driving_v_err = node_traffic_rule["backward_driving_v_err"].as<double>();
    activated_rules = node_traffic_rule["activated_rules"].as<std::vector<std::string>>();
    mode_spot = node_traffic_rule["mode_spot"].as<int>();
    mode_automata = node_traffic_rule["mode_automata"].as<int>();
    dis_stop_line = node_traffic_rule["dis_stop_line"].as<double>();
}

SemanticConfiguration::SemanticConfiguration(YAML::Node const &node) : reach::Configuration(node) {
    config_traffic_rule = TrafficRuleConfiguration(node);
}

SemanticConfigurationPtr SemanticConfiguration::load_configuration(string const &file_yaml) {
    YAML::Node node = YAML::LoadFile(file_yaml);
    auto config = SemanticConfiguration(node);
    return make_shared<SemanticConfiguration>(node);
}
