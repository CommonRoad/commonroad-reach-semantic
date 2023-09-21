#pragma once

#include <pybind11/embed.h>

#include "reachset/data_structure/reach/reach_polygon.hpp"
#include "reachset/data_structure/reach/reach_node.hpp"
#include "reach_semantic/data_structure/region.hpp"
#include "reach_semantic/data_structure/environment_model/semantic_model.hpp"
#include "reach_semantic/data_structure/position_interval.hpp"

namespace semantic_reach{
    /// Splits reachable sets and labels the parts according to the semantic model.
    class ReachableSetLabeler {
    private:
        /// Python handle for semantic model
        SemanticModelPtr semantic_model;
        SemanticConfigurationPtr config;

        /// Returns the propositions of the given rectangle.
        ///
        /// Intersects the rectangle with regions and position intervals. Since this method does not split the rectangle,
        /// it adds the propositions of the first intersecting region and position interval.
        std::pair<PropositionHolder, std::set<int>> _obtain_propositions_for_rectangle(const reach::ReachPolygonPtr& rectangle, int step);

        /// Labels propagated sets with traffic status propositions.
        std::vector<reach::ReachNodePtr> _label_traffic_status_propositions(int step, std::vector<reach::ReachNodePtr> reachable_sets);

        /// Labels propagated sets with propositions related to conflict status between them and the vehicles.
        ///
        /// Note: this relationship is asymmetric, refer to Sebastian's intersection traffic rule paper for definition.
        /// A lanelet is examined against a list of lanelets of the lane/route of the other object.
        std::vector<reach::ReachNodePtr> _label_in_conflict_area_propositions(int step, std::vector<reach::ReachNodePtr> reachable_sets);

        /// Labels propagated sets with propositions related to causes braking to other vehicles.
        std::vector<reach::ReachNodePtr> _label_causes_braking_propositions(int step, std::vector<reach::ReachNodePtr> reachable_sets);

        /// Updates the propositions of the propagated set with the proposition of the lanelet region.
        ///
        /// Lanelet transition proposition are considered to be temporary since they are only used for TPL compliance
        /// checking and should be omitted during merging of the propagated sets.
        reach::ReachNodePtr _update_propositions_with_region(reach::ReachNodePtr propagated_set, const RegionPtr& region, int step);

        std::vector<reach::ReachNodePtr> _split_reachable_set_wrt_intervals(const reach::ReachNodePtr& reachable_set, const std::vector<PositionIntervalPtr>& intervals, double reach_min, double reach_max, const std::string& direction);

        /// Returns the set of lanelet transition propositions.
        std::set<std::string> _obtain_lanelet_transition_propositions(const reach::ReachNodePtr& propagated_set);

    public:
        std::map<reach::ReachNodePtr, PropositionHolder> reachable_set_to_propositions;
        std::map<reach::ReachNodePtr, std::set<int>> reachable_set_to_lanelet_ids;
        ReachableSetLabeler(SemanticModelPtr semantic_model_py_obj, SemanticConfigurationPtr config);

        /// Assigns proposition labels to initial reachable sets and drivable areas.
        void label_initial_state(const std::vector<reach::ReachNodePtr>& reachable_sets, int step_start);

        /// Labels propagated sets with propositions related to traffic status.
        std::vector<reach::ReachNodePtr> label_traffic_propositions(int step, std::vector<reach::ReachNodePtr> reachable_sets);

        /// Splits a reachable set w.r.t lanelet regions.
        ///
        /// Steps:
        ///   1. Intersect reachable set in the position domain with lanelet regions
        ///   2. Over-approximate and restore to axis-aligned rectangles
        std::vector<reach::ReachNodePtr> split_wrt_regions(int step, const std::vector<reach::ReachNodePtr> &reachable_sets);

        /// Splits the reachable set w.r.t position intervals.
        std::vector<reach::ReachNodePtr> split_wrt_position_intervals(int step, const std::vector<reach::ReachNodePtr>& reachable_sets);

        /// Returns a list of propagated sets that do not collide with vehicles.
        std::vector<reach::ReachNodePtr> discard_colliding_nodes(const std::vector<reach::ReachNodePtr>& reachable_sets);

        /// Copy the labels of source_reachable_set to every node in reachable_sets.
        void copy_labels(const reach::ReachNodePtr& source_reachable_set, const std::vector<reach::ReachNodePtr>& reachable_sets);
    };

    using ReachableSetLabelerPtr = std::shared_ptr<ReachableSetLabeler>;
}