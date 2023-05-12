#pragma once

#include "reach_semantic/utility/shared_include.hpp"
#include "reach_semantic/data_structure/semantic_configuration.hpp"
#include "reach_semantic/data_structure/traffic_rule.hpp"
#include "reach_semantic/data_structure/region.hpp"
#include "reach_semantic/data_structure/position_interval.hpp"

namespace semantic_reach {
/// Class to represent the semantic model of a given CommonRoad scenario.
class SemanticModel {
public:
    py::handle obj_semantic_model_py;

    SemanticConfigurationPtr config;
    int step_start{};
    int step_end{};

    // lanelet region
    std::vector<RegionPtr> vec_regions;
    // vehicle position interval
    std::map<int, std::map<std::string, std::vector<PositionIntervalPtr>>> map_step_to_position_intervals;

    std::map<int, std::set<std::string>> map_step_to_traffic_status_propositions;

    explicit SemanticModel(py::handle const& obj_semantic_model_py);
};

using SemanticModelPtr = shared_ptr<SemanticModel>;
}