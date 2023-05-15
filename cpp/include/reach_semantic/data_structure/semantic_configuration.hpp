#pragma once

#include <yaml-cpp/yaml.h>

#include "reachset/data_structure/configuration.hpp"
#include "reach_semantic/utility/shared_include.hpp"
#include "reachset/utility/shared_using.hpp"

namespace semantic_reach {

/// Struct storing reachable set configurations.
struct SemanticReachableSetConfiguration : reach::ReachableSetConfiguration {
    // shortest length the edges of a reachable node should possess for it to be kept
    double length_edge_node_min{};

    SemanticReachableSetConfiguration() = default;

    explicit SemanticReachableSetConfiguration(YAML::Node const& node);
};

struct SemanticModelConfiguration {
    bool is_intersection{};
    double ego_radius_inflation{};
    std::vector<int> vec_route_lanelet_ids{};

    SemanticModelConfiguration() = default;
};

/// Struct storing all configurations.
struct SemanticConfiguration : reach::Configuration {
    SemanticReachableSetConfiguration config_reachable_set{};
    SemanticModelConfiguration config_semantic_model{};

    SemanticConfiguration() = default;

    explicit SemanticConfiguration(YAML::Node const& node);

    inline SemanticReachableSetConfiguration& reachable_set() { return config_reachable_set; };
    inline SemanticModelConfiguration& semantic_model() { return config_semantic_model; };

    /// Loads configuration from the given yaml file.
    static std::shared_ptr<SemanticConfiguration> load_configuration(std::string const& file_yaml);
};

using SemanticConfigurationPtr = std::shared_ptr<SemanticConfiguration>;
}

