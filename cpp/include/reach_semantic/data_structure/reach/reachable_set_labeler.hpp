#pragma once

#include <pybind11/embed.h>

#include "reachset/data_structure/reach/reach_polygon.hpp"
#include "reach_semantic/data_structure/region.hpp"
#include "reach_semantic/data_structure/semantic_model.hpp"
#include "reach_semantic/data_structure/position_interval.hpp"
#include "reach_semantic/data_structure/reach/semantic_reach_node.hpp"

namespace semantic_reach{
    /// Splits reachable sets and labels the parts according to the semantic model.
    class ReachableSetLabeler {
    private:
        /// Python handle for semantic model
        SemanticModelPtr semantic_model;

        /// Returns the propositions of the given rectangle.
        ///
        /// Intersects the rectangle with regions and position intervals. Since this method does not split the rectangle,
        /// it adds the propositions of the first intersecting region and position interval.
        std::pair<PropositionHolder, std::set<int>> _obtain_propositions_for_rectangle(const reach::ReachPolygonPtr& rectangle, int step);

        /// Labels propagated sets with traffic status propositions.
        std::vector<SemanticReachNodePtr> _label_traffic_status_propositions(int step, std::vector<SemanticReachNodePtr> reachable_sets);

        /// Labels propagated sets with propositions related to conflict status between them and the vehicles.
        ///
        /// Note: this relationship is asymmetric, refer to Sebastian's intersection traffic rule paper for definition.
        /// A lanelet is examined against a list of lanelets of the lane/route of the other object.
        std::vector<SemanticReachNodePtr> _label_in_conflict_area_propositions(int step, std::vector<SemanticReachNodePtr> reachable_sets);

        /// Labels propagated sets with propositions related to causes braking to other vehicles.
        std::vector<SemanticReachNodePtr> _label_causes_braking_propositions(int step, std::vector<SemanticReachNodePtr> reachable_sets);

        /// Updates the propositions of the propagated set with the proposition of the lanelet region.
        ///
        /// Lanelet transition proposition are considered to be temporary since they are only used for TPL compliance
        /// checking and should be omitted during merging of the propagated sets.
        SemanticReachNodePtr _update_propositions_with_region(SemanticReachNodePtr propagated_set, const RegionPtr& region, int step);

        std::vector<SemanticReachNodePtr> _split_reachable_set_wrt_intervals(const SemanticReachNodePtr& reachable_set, const std::vector<PositionIntervalPtr>& intervals, double reach_min, double reach_max, const std::string& direction);

        /// Returns the set of lanelet transition propositions.
        std::set<std::string> _obtain_lanelet_transition_propositions(const SemanticReachNodePtr& propagated_set);

    public:
        std::map<SemanticReachNodePtr, PropositionHolder> reachable_set_to_propositions;
        std::map<SemanticReachNodePtr, std::set<int>> reachable_set_to_lanelet_ids;
        ReachableSetLabeler(SemanticModelPtr semantic_model_py_obj);

        /// Assigns proposition labels to initial reachable sets and drivable areas.
        void label_initial_state(const std::vector<SemanticReachNodePtr>& reachable_sets, int step_start);

        /// Labels propagated sets with propositions related to traffic status.
        std::vector<SemanticReachNodePtr> label_traffic_propositions(int step, std::vector<SemanticReachNodePtr> reachable_sets);

        /// Splits a reachable set w.r.t lanelet regions.
        ///
        /// Steps:
        ///   1. Intersect reachable set in the position domain with lanelet regions
        ///   2. Over-approximate and restore to axis-aligned rectangles
        std::vector<SemanticReachNodePtr> split_wrt_regions(int step, const std::vector<SemanticReachNodePtr> &reachable_sets);

        /// Splits the reachable set w.r.t position intervals.
        std::vector<SemanticReachNodePtr> split_wrt_position_intervals(int step, std::vector<SemanticReachNodePtr> reachable_sets);

        /// Returns a list of propagated sets that do not collide with vehicles.
        std::vector<SemanticReachNodePtr> discard_colliding_nodes(std::vector<SemanticReachNodePtr> reachable_sets);

        /// Copy the labels of source_reachable_set to every node in reachable_sets.
        void copy_labels(const SemanticReachNodePtr& source_reachable_set, const std::vector<SemanticReachNodePtr>& reachable_sets);
    };

    using ReachableSetLabelerPtr = std::unique_ptr<ReachableSetLabeler>;
}