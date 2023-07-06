#include "reach_semantic/data_structure/semantic_configuration.hpp"
#include "reachset/utility/shared_using.hpp"

using namespace semantic_reach;

SemanticReachableSetConfiguration::SemanticReachableSetConfiguration(YAML::Node const& node) : reach::ReachableSetConfiguration(node) {
    auto node_reachable_set = node["reachable_set"];

    discard_small_nodes = node_reachable_set["discard_small_nodes"].as<bool>();
    length_edge_node_min = node_reachable_set["length_edge_node_min"].as<double>();
}

TrafficRuleConfiguration::TrafficRuleConfiguration(const YAML::Node &node) {
    auto node_traffic_rule = node["traffic_rule"];

    distance_braking = node_traffic_rule["distance_braking"].as<double>();
    acceleration_braking_hard = node_traffic_rule["acceleration_braking_hard"].as<double>();
    backward_driving_v_err = node_traffic_rule["backward_driving_v_err"].as<double>();
    activated_rules = node_traffic_rule["activated_rules"].as<std::vector<std::string>>();
    mode_spot = node_traffic_rule["mode_spot"].as<int>();
    mode_automata = node_traffic_rule["mode_automata"].as<int>();
}

SemanticConfiguration::SemanticConfiguration(YAML::Node const& node) : reach::Configuration(node) {
    config_reachable_set = SemanticReachableSetConfiguration(node);
    config_traffic_rule = TrafficRuleConfiguration(node);
}

SemanticConfigurationPtr SemanticConfiguration::load_configuration(string const& file_yaml) {
    YAML::Node node = YAML::LoadFile(file_yaml);
    auto config = SemanticConfiguration(node);
    return make_shared<SemanticConfiguration>(node);
}
