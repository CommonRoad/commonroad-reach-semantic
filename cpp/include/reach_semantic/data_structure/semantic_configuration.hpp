#pragma once

#include <yaml-cpp/yaml.h>

#include "reach_semantic/utility/shared_include.hpp"
#include "reachset/data_structure/configuration.hpp"
#include "reachset/utility/shared_using.hpp"

namespace semantic_reach {

/// Struct storing general configurations.
struct GeneralConfiguration {
    std::string name_scenario{};
    std::string path_scenarios{};
    std::string path_scenario{};

    GeneralConfiguration() = default;

    [[nodiscard]] reach::GeneralConfiguration as_reach_config() const;

    explicit GeneralConfiguration(YAML::Node const &node);
};

/// Struct storing traffic rule configurations.
struct TrafficRuleConfiguration : reach::ReachableSetConfiguration {
    double distance_braking{};
    double acceleration_braking_hard{};
    double backward_driving_v_err{};
    double dis_stop_line{};
    double fov_speed_limit{};
    double braking_speed_limit{};
    std::vector<std::string> activated_rules{};
    int mode_spot{};
    int mode_automata{};

    TrafficRuleConfiguration() = default;

    explicit TrafficRuleConfiguration(YAML::Node const &node);
};

struct SemanticModelConfiguration {
    bool is_intersection{};
    double ego_radius_inflation{};
    std::vector<int> vec_route_lanelet_ids{};

    SemanticModelConfiguration() = default;
};

/// Struct storing all configurations.
struct SemanticConfiguration {
    GeneralConfiguration config_general{};
    reach::VehicleConfiguration config_vehicle{};
    reach::PlanningConfiguration config_planning{};
    reach::ReachableSetConfiguration config_reachable_set{};
    reach::DebugConfiguration config_debug{};
    TrafficRuleConfiguration config_traffic_rule{};
    SemanticModelConfiguration config_semantic_model{};

    SemanticConfiguration() = default;

    [[nodiscard]] reach::ConfigurationPtr as_reach_config() const;

    explicit SemanticConfiguration(YAML::Node const &node);

    inline GeneralConfiguration &general() { return config_general; };

    inline reach::VehicleConfiguration &vehicle() { return config_vehicle; };

    inline reach::PlanningConfiguration &planning() { return config_planning; };

    inline reach::ReachableSetConfiguration &reachable_set() { return config_reachable_set; };

    inline reach::DebugConfiguration &debug() { return config_debug; };

    inline SemanticModelConfiguration &semantic_model() { return config_semantic_model; };

    /// Loads configuration from the given yaml file.
    static std::shared_ptr<SemanticConfiguration> load_configuration(std::string const &file_yaml);
};

using SemanticConfigurationPtr = std::shared_ptr<SemanticConfiguration>;
} // namespace semantic_reach
