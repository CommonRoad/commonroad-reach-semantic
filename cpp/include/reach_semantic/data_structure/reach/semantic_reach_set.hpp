#pragma once

#include "reach_semantic/utility/shared_include.hpp"
#include "reachset/data_structure/reach/reach_polygon.hpp"
#include "reachset/data_structure/reach/reach_node.hpp"
#include "reach_semantic/data_structure/reach/reachable_set_labeler.hpp"
#include "reach_semantic/data_structure/semantic_configuration.hpp"
#include "reach_semantic/data_structure/semantic_model.hpp"
#include "reach_semantic/data_structure/proposition.hpp"
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
class SemanticReachableSet {
private:
    bool _reachable_set_computed{false};
    bool _pruned{false};
    std::vector<int> _vec_steps_computed{};

    void _initialize();

    std::vector<reach::ReachNodePtr> _construct_initial_reachable_sets();

    void _initialize_zero_state_polygons();

    void _compute_drivable_area_at_step(int const& step);

    void _compute_reachable_set_at_step(int const& step);

    /// Propagates the nodes of the reachable set.
    std::vector<reach::ReachNodePtr> _propagate_reachable_set(std::vector<reach::ReachNodePtr> const& vec_nodes);

    /// Splits propagated sets w.r.t lanelet regions.
    std::vector<reach::ReachNodePtr> _split_wrt_regions(int const& step, std::vector<reach::ReachNodePtr> const& vec_nodes);

    /// Splits the propagated sets w.r.t position intervals.
    std::vector<reach::ReachNodePtr> _split_wrt_intervals(int const& step, std::vector<reach::ReachNodePtr> const& vec_nodes);

    std::vector<reach::ReachNodePtr> _discard_colliding_nodes(std::vector<reach::ReachNodePtr> const& vec_nodes);

    /// Computes collision free drivable area.
    std::vector<reach::ReachPolygonPtr>
    _collision_check_and_repartition(std::vector<reach::ReachPolygonPtr> rectangles, int const &step);

    /// Dummy function for computing the overhead of calling python functions.
    std::vector<reach::ReachNodePtr> _call_python_dummy(int const& step, vector<reach::ReachNodePtr> const& vec_nodes);

public:
    explicit SemanticReachableSet(SemanticConfigurationPtr config);

    SemanticReachableSet(SemanticConfigurationPtr config, CollisionCheckerPtr collision_checker, SemanticModelPtr semantic_model);

    SemanticReachableSet(SemanticConfigurationPtr config, CollisionCheckerPtr collision_checker,
                 SemanticModelPtr semantic_model, TrafficRuleInterfacePtr traffic_rule_interface);

    SemanticConfigurationPtr config;
    CollisionCheckerPtr collision_checker;
    ReachableSetLabelerPtr labeler;
    SemanticModelPtr semantic_model;
    TrafficRuleInterfacePtr rule_interface;

    int step_start{};
    int step_end{};

    std::map<int, std::vector<reach::ReachNodePtr>> map_step_to_reachable_set{};
    std::map<int, std::vector<reach::ReachPolygonPtr>> map_step_to_drivable_area{};
    std::map<int, std::vector<reach::ReachNodePtr>> map_step_to_propagated_set{};

    std::map<int, std::unordered_map<PropositionHolder, std::vector<reach::ReachNodePtr>, PropositionHolder::HashFunction>>
            map_step_to_propositions_to_propagated_set{};
    std::map<int, std::unordered_map<PropositionHolder, std::vector<reach::ReachPolygonPtr>, PropositionHolder::HashFunction>>
            map_step_to_propositions_to_drivable_area{};

    reach::ReachPolygonPtr polygon_zero_state_lon;
    reach::ReachPolygonPtr polygon_zero_state_lat;

    /// Returns the propositions of the given rectangle.
    PropositionHolder obtain_propositions_for_rectangle(reach::ReachPolygonPtr const& rectangle, int const& step) const;

    inline reach::ReachNodePtr update_propositions_with_region(reach::ReachNodePtr const& node,
                                                        RegionPtr const& region, int const& step);

    void compute(int step_start = 0, int step_end = 0);

    //void prune_nodes_not_reaching_final_step();

    inline std::map<int, std::vector<reach::ReachPolygonPtr>> drivable_area() const { return map_step_to_drivable_area; }
    inline std::map<int, std::vector<reach::ReachNodePtr>> reachable_set() const { return map_step_to_reachable_set; }
    inline std::map<int, std::vector<reach::ReachNodePtr>> propagated_set() const { return map_step_to_propagated_set; }

    inline std::vector<reach::ReachPolygonPtr>
    drivable_area_at_step(int const& step) {
        if (find(_vec_steps_computed.begin(), _vec_steps_computed.end(), step)
            == _vec_steps_computed.end()) {
            cout << "Given step " << step << "for drivable area retrieval is out of range." << endl;
            return {};

        } else {
            return map_step_to_drivable_area[step];
        }
    }

    inline std::vector<reach::ReachNodePtr>
    reachable_set_at_step(int const& step) {
        if (find(_vec_steps_computed.begin(), _vec_steps_computed.end(), step)
            == _vec_steps_computed.end()) {
            cout << "Given step " << step << "for reachable set retrieval is out of range." << endl;
            return {};

        } else {
            return map_step_to_reachable_set[step];
        }
    }
};

using SemanticReachableSetPtr = shared_ptr<SemanticReachableSet>;
}