#pragma once

#include "reachset/utility/shared_include.hpp"
#include "reachset/data_structure/configuration.hpp"
#include "reachset/data_structure/traffic_rule.hpp"
#include "reachset/data_structure/region.hpp"
#include "reachset/data_structure/position_interval.hpp"

namespace reach {
/// Class to represent the semantic model of a given CommonRoad scenario.
class SemanticModel {
public:
    py::handle obj_semantic_model_py;

    ConfigurationPtr config;
    int step_start{};
    int step_end{};

    // lanelet region
    std::vector<RegionPtr> vec_regions;
    // vehicle position interval
    std::map<int, std::map<std::string, std::vector<PositionIntervalPtr>>> map_step_to_position_intervals;

    explicit SemanticModel(py::handle const& obj_semantic_model_py);
};

using SemanticModelPtr = shared_ptr<SemanticModel>;
}