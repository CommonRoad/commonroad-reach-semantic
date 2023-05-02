#include "reach_semantic/data_structure/semantic_configuration.hpp"
#include "reachset/utility/shared_using.hpp"

using namespace semantic_reach;

SemanticReachableSetConfiguration::SemanticReachableSetConfiguration(YAML::Node const& node) : reach::ReachableSetConfiguration(node) {
    auto node_reachable_set = node["reachable_set"];

    length_edge_node_min = node_reachable_set["length_edge_node_min"].as<double>();
}

SemanticConfiguration::SemanticConfiguration(YAML::Node const& node) : reach::Configuration(node) {
    config_reachable_set = SemanticReachableSetConfiguration(node);
}

SemanticConfigurationPtr SemanticConfiguration::load_configuration(string const& file_yaml) {
    YAML::Node node = YAML::LoadFile(file_yaml);
    auto config = SemanticConfiguration(node);
    return make_shared<SemanticConfiguration>(node);
}
