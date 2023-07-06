#pragma once

#include "collision/collision_checker.h"
#include "reachset/data_structure/reach/reach_polygon.hpp"
#include "reachset/data_structure/reach/reach_node.hpp"
#include "reach_semantic/data_structure/reach/reachable_set_labeler.hpp"
#include "reach_semantic/data_structure/semantic_configuration.hpp"
#include "reach_semantic/data_structure/semantic_model.hpp"
#include "reach_semantic/benchmark/benchmark_result.hpp"

namespace semantic_reach {
    class SemanticReachableSet {
    protected:
        bool _reachable_set_computed{false};
        bool _pruned{false};
        std::vector<int> _vec_steps_computed{};

        std::vector<reach::ReachNodePtr> _construct_initial_reachable_sets();

        void _initialize_zero_state_polygons();

        /// Propagates the nodes of the reachable set.
        std::vector<reach::ReachNodePtr> _propagate_reachable_set(std::vector<reach::ReachNodePtr> const &vec_nodes);

        /// Computes collision free drivable area.
        std::vector<reach::ReachPolygonPtr>
        _collision_check_and_repartition(std::vector<reach::ReachPolygonPtr> rectangles, int const &step);

        virtual void _compute_drivable_area_at_step(int const &step) = 0;

        virtual void _compute_reachable_set_at_step(int const &step) = 0;

    public:
        SemanticReachableSet(SemanticConfigurationPtr config, collision::CollisionCheckerPtr collision_checker,
                             SemanticModelPtr semantic_model, TrafficRuleInterfacePtr traffic_rule_interface);

        ReachBenchmarkResults benchmark_result{};

        SemanticConfigurationPtr config;
        collision::CollisionCheckerPtr collision_checker;
        ReachableSetLabelerPtr labeler;
        SemanticModelPtr semantic_model;
        TrafficRuleInterfacePtr rule_interface;

        virtual ~SemanticReachableSet() = default;

        int step_start{};
        int step_end{};

        std::map<int, std::vector<reach::ReachNodePtr>> map_step_to_reachable_set{};
        std::map<int, std::vector<reach::ReachPolygonPtr>> map_step_to_drivable_area{};
        std::map<int, std::vector<reach::ReachNodePtr>> map_step_to_propagated_set{};

        reach::ReachPolygonPtr polygon_zero_state_lon;
        reach::ReachPolygonPtr polygon_zero_state_lat;

        void compute(int step_start = 0, int step_end = 0);

        void prune_nodes_not_reaching_final_step();

        inline std::map<int, std::vector<reach::ReachPolygonPtr>>
        drivable_area() const { return map_step_to_drivable_area; }

        inline std::map<int, std::vector<reach::ReachNodePtr>>
        reachable_set() const { return map_step_to_reachable_set; }

        inline std::map<int, std::vector<reach::ReachNodePtr>>
        propagated_set() const { return map_step_to_propagated_set; }

        inline std::vector<reach::ReachPolygonPtr>
        drivable_area_at_step(int const &step) {
            if (find(_vec_steps_computed.begin(), _vec_steps_computed.end(), step)
                == _vec_steps_computed.end()) {
                cout << "Given step " << step << "for drivable area retrieval is out of range." << endl;
                return {};

            } else {
                return map_step_to_drivable_area[step];
            }
        }

        inline std::vector<reach::ReachNodePtr>
        reachable_set_at_step(int const &step) {
            if (find(_vec_steps_computed.begin(), _vec_steps_computed.end(), step)
                == _vec_steps_computed.end()) {
                cout << "Given step " << step << "for reachable set retrieval is out of range." << endl;
                return {};

            } else {
                return map_step_to_reachable_set[step];
            }
        }
    };
}