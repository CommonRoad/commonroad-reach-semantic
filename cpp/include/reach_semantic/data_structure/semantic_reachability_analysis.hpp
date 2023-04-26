#pragma once
#include "reach_semantic/common/utility/shared_include.hpp"
#include "reach_semantic/common/data_structure/reach_polygon_boost.hpp"
#include "reach_semantic/common/data_structure/reach_node.hpp"
#include "reach_semantic/common/data_structure/configuration.hpp"
#include "reach_semantic/semantic/data_structure/semantic_model.hpp"
#include "collision/collision_checker.h"
#include <omp.h>

using CollisionCheckerPtr = collision::CollisionCheckerPtr;

namespace reach {
/// Semantic reachability analysis of vehicles.
class SemanticReachabilityAnalysis {
public:
    std::set<std::string> obtain_propositions_for_rectangle(ReachPolygonPtr const& rectangle) const;

    std::tuple<std::map<std::set<std::string>, std::vector<ReachPolygonPtr>>,
            std::map<std::set<std::string>, std::vector<ReachNodePtr>>> compute_drivable_area_at_time_step(
            int const& time_step,
            std::map<std::set<std::string>, std::vector<ReachNodePtr>> const& reachable_set_time_step_previous);

    /// Propagates the nodes of the reachable set from the last time step.
    std::vector<ReachNodePtr> propagate_reachable_set(const std::vector<ReachNodePtr>& vec_nodes);

    std::vector<ReachNodePtr> adapt_base_sets_to_regions(std::vector<ReachNodePtr> const& vec_base_sets) const;

    std::vector<ReachNodePtr>
    adapt_base_sets_to_vehicles(int const& time_step, std::vector<ReachNodePtr> const& vec_base_sets) const;

    static ReachNodePtr adapt_base_set_to_interval(
            ReachNodePtr const& base_set, PositionIntervalPtr const& interval, std::string const& direction);

    static std::vector<ReachNodePtr> discard_colliding_base_sets(std::vector<ReachNodePtr> const& vec_base_sets);

    std::map<std::set<std::string>, std::vector<ReachNodePtr>> compute_reachable_set_at_time_step(
            int const& time_step,
            std::map<std::set<std::string>, std::vector<ReachNodePtr>> & map_propositions_to_vec_base_sets_propagated,
            std::map<std::set<std::string>, std::vector<ReachPolygonPtr>> const& map_propositions_to_drivable_area);
};

using SemanticReachabilityAnalysisPtr = shared_ptr<SemanticReachabilityAnalysis>;
}