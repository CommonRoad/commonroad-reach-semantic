#pragma once

#include "reach_semantic/utility/shared_include.hpp"
#include "reachset/data_structure/reach/reach_polygon.hpp"
#include "reach_semantic/data_structure/reach/semantic_reach_node.hpp"
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

    void _construct_initial_drivable_area_and_reachable_set();

    void _initialize_zero_state_polygons();

    void _compute_drivable_area_at_step(int const& step);

    void _compute_reachable_set_at_step(int const& step);

    /// Propagates the nodes of the reachable set.
    std::vector<SemanticReachNodePtr> _propagate_reachable_set(std::vector<SemanticReachNodePtr> const& vec_nodes);

    /// Splits propagated sets w.r.t lanelet regions.
    std::vector<SemanticReachNodePtr> _split_wrt_regions(int const& step, std::vector<SemanticReachNodePtr> const& vec_nodes);

    /// Splits the propagated sets w.r.t position intervals.
    std::vector<SemanticReachNodePtr> _split_wrt_intervals(int const& step, std::vector<SemanticReachNodePtr> const& vec_nodes);

    std::vector<SemanticReachNodePtr> _discard_colliding_nodes(std::vector<SemanticReachNodePtr> const& vec_nodes);

    /// Computes collision free drivable area.
    std::unordered_map<PropositionHolder, std::vector<reach::ReachPolygonPtr>, PropositionHolder::HashFunction>
    _compute_collision_free_drivable_area(int const& step,
                                          std::unordered_map<PropositionHolder, std::vector<SemanticReachNodePtr>,
                                                  PropositionHolder::HashFunction> const&
                                          map_propositions_to_drivable_area);

    /// Dummy function for computing the overhead of calling python functions.
    std::vector<SemanticReachNodePtr> _call_python_dummy(int const& step, vector<SemanticReachNodePtr> const& vec_nodes);

public:
    explicit SemanticReachableSet(SemanticConfigurationPtr config);

    SemanticReachableSet(SemanticConfigurationPtr config, CollisionCheckerPtr collision_checker, SemanticModelPtr semantic_model);

    SemanticReachableSet(SemanticConfigurationPtr config, CollisionCheckerPtr collision_checker,
                 SemanticModelPtr semantic_model, TrafficRuleInterfacePtr traffic_rule_interface);

    SemanticConfigurationPtr config;
    CollisionCheckerPtr collision_checker;
    SemanticModelPtr semantic_model;
    TrafficRuleInterfacePtr rule_interface;

    int step_start{};
    int step_end{};

    std::map<int, std::unordered_map<PropositionHolder, std::vector<SemanticReachNodePtr>, PropositionHolder::HashFunction>>
            map_step_to_propositions_to_propagated_set{};
    std::map<int, std::unordered_map<PropositionHolder, std::vector<reach::ReachPolygonPtr>, PropositionHolder::HashFunction>>
            map_step_to_propositions_to_drivable_area{};
    std::map<int, std::unordered_map<PropositionHolder, std::vector<SemanticReachNodePtr>, PropositionHolder::HashFunction>>
            map_step_to_propositions_to_reachable_set{};

    reach::ReachPolygonPtr polygon_zero_state_lon;
    reach::ReachPolygonPtr polygon_zero_state_lat;

    /// Returns the propositions of the given rectangle.
    PropositionHolder obtain_propositions_for_rectangle(reach::ReachPolygonPtr const& rectangle, int const& step) const;

    /// Label traffic propositions using Python script.
    inline std::vector<SemanticReachNodePtr> label_traffic_propositions(int const& step, std::vector<SemanticReachNodePtr> vec_nodes);

    inline SemanticReachNodePtr update_propositions_with_region(SemanticReachNodePtr const& node,
                                                        RegionPtr const& region, int const& step);

    void compute(int step_start = 0, int step_end = 0);

    //void prune_nodes_not_reaching_final_step();

    inline std::unordered_map<PropositionHolder, std::vector<reach::ReachPolygonPtr>, PropositionHolder::HashFunction>
    drivable_area_at_step(int const& step) {
        if (find(_vec_steps_computed.begin(), _vec_steps_computed.end(), step)
            == _vec_steps_computed.end()) {
            cout << "Given step " << step << "for drivable area retrieval is out of range." << endl;
            return {};

        } else {
            return map_step_to_propositions_to_drivable_area[step];
        }
    }

    inline std::vector<reach::ReachPolygonPtr> drivable_area_merge_at_step(int const& step) {
        if (find(_vec_steps_computed.begin(), _vec_steps_computed.end(), step)
            == _vec_steps_computed.end()) {
            cout << "Given step " << step << "for drivable area retrieval is out of range." << endl;
            return {};

        } else {
            std::vector<reach::ReachPolygonPtr> vec_drivable_area_merged{};
            auto map_propositions_to_drivable_area = map_step_to_propositions_to_drivable_area[step];

            for (auto const& [proposition, vec_drivable_area]:
                    map_propositions_to_drivable_area) {
                vec_drivable_area_merged.insert(vec_drivable_area_merged.end(),
                                                std::make_move_iterator(vec_drivable_area.begin()),
                                                std::make_move_iterator(vec_drivable_area.end()));
            }

            return vec_drivable_area_merged;
        }
    }

    inline std::unordered_map<PropositionHolder, std::vector<SemanticReachNodePtr>, PropositionHolder::HashFunction>
    reachable_set_at_step(int const& step) {
        if (find(_vec_steps_computed.begin(), _vec_steps_computed.end(), step)
            == _vec_steps_computed.end()) {
            cout << "Given step " << step << "for reachable set retrieval is out of range." << endl;
            return {};

        } else {
            return map_step_to_propositions_to_reachable_set[step];
        }
    }

    inline std::vector<SemanticReachNodePtr> reachable_set_merge_at_step(int const& step) {
        if (find(_vec_steps_computed.begin(), _vec_steps_computed.end(), step)
            == _vec_steps_computed.end()) {
            cout << "Given step " << step << "for drivable area retrieval is out of range." << endl;
            return {};

        } else {
            std::vector<SemanticReachNodePtr> vec_reachable_set_merged{};
            auto map_propositions_to_reachable_set = map_step_to_propositions_to_reachable_set[step];

            for (auto const& [proposition, vec_reachable_set]:
                    map_propositions_to_reachable_set) {
                vec_reachable_set_merged.insert(vec_reachable_set_merged.end(),
                                                std::make_move_iterator(vec_reachable_set.begin()),
                                                std::make_move_iterator(vec_reachable_set.end()));
            }

            return vec_reachable_set_merged;
        }
    }

};

using SemanticReachableSetPtr = shared_ptr<SemanticReachableSet>;
}