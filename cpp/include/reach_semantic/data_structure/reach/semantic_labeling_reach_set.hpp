#pragma once

#include "reach_semantic/utility/shared_include.hpp"
#include "reachset/data_structure/reach/reach_polygon.hpp"
#include "reachset/data_structure/reach/reach_node.hpp"
#include "reach_semantic/data_structure/reach/reachable_set_labeler.hpp"
#include "reach_semantic/data_structure/semantic_configuration.hpp"
#include "reach_semantic/data_structure/semantic_model.hpp"
#include "reach_semantic/data_structure/proposition.hpp"
#include "reach_semantic/data_structure/reach/semantic_reach_set.hpp"
#include "collision/collision_checker.h"
#include <omp.h>

#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>

using CollisionCheckerPtr = collision::CollisionCheckerPtr;
namespace py = pybind11;

namespace semantic_reach {
/// Reachable set representation for the ego vehicle.
class SemanticLabelingReachableSet : public SemanticReachableSet {
private:
    void _compute_drivable_area_at_step(int const& step);

    void _compute_reachable_set_at_step(int const& step);

public:
    SemanticLabelingReachableSet(SemanticConfigurationPtr config, CollisionCheckerPtr collision_checker,
                 SemanticModelPtr semantic_model, TrafficRuleInterfacePtr traffic_rule_interface);

    std::map<int, std::unordered_map<PropositionHolder, std::vector<reach::ReachNodePtr>, PropositionHolder::HashFunction>>
            map_step_to_propositions_to_propagated_set{};
    std::map<int, std::unordered_map<PropositionHolder, std::vector<reach::ReachPolygonPtr>, PropositionHolder::HashFunction>>
            map_step_to_propositions_to_drivable_area{};
};

using SemanticLabelingReachableSetPtr = shared_ptr<SemanticLabelingReachableSet>;
}