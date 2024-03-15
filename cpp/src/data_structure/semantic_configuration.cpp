#include "reach_semantic/data_structure/semantic_configuration.hpp"
#include "reachset/utility/shared_using.hpp"

using namespace semantic_reach;

GeneralConfiguration::GeneralConfiguration(const YAML::Node &node) {
    name_scenario = node["name_scenario"].as<string>();
    path_scenarios = node["path_scenarios"].as<string>();
    path_scenario = node["path_scenario"].as<string>();
}

reach::GeneralConfiguration GeneralConfiguration::as_reach_config() const {
    auto config = reach::GeneralConfiguration();
    config.name_scenario = name_scenario;
    config.path_scenarios = path_scenarios;
    config.path_scenario = path_scenario;
    return config;
}

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

SemanticConfiguration::SemanticConfiguration(YAML::Node const &node) {
    auto reach_config{reach::Configuration(node)};
    config_general = GeneralConfiguration(node);
    config_vehicle = reach_config.config_vehicle;
    config_planning = reach_config.config_planning;
    config_reachable_set = reach_config.config_reachable_set;
    config_debug = reach_config.config_debug;
    config_traffic_rule = TrafficRuleConfiguration(node);
}

reach::ConfigurationPtr SemanticConfiguration::as_reach_config() const {
    auto config = std::make_shared<reach::Configuration>();
    config->config_general = config_general.as_reach_config();
    config->config_vehicle = config_vehicle;
    config->config_planning = config_planning;
    config->config_reachable_set = config_reachable_set;
    config->config_debug = config_debug;
    return config;
}

SemanticConfigurationPtr SemanticConfiguration::load_configuration(string const &file_yaml) {
    YAML::Node node = YAML::LoadFile(file_yaml);
    auto config = SemanticConfiguration(node);
    return make_shared<SemanticConfiguration>(node);
}
