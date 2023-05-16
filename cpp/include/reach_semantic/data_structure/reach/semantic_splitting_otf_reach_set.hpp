#pragma once

#include "reach_semantic/data_structure/reach/semantic_reach_set.hpp"
#include "reach_semantic/data_structure/model_checking/finite_automaton.hpp"

namespace semantic_reach {
    class SemanticSplittingOTFReachableSet : public SemanticReachableSet {
    private:
        void _compute_drivable_area_at_step(int const &step) override;

        void _compute_reachable_set_at_step(int const &step) override;

        std::map<int, std::map<std::set<unsigned int>, std::vector<reach::ReachPolygonPtr>>> map_step_to_states_to_drivable_area{};
        std::map<int, std::map<std::set<unsigned int>, std::vector<reach::ReachNodePtr>>> map_step_to_states_to_propagated_set{};
        std::map<reach::ReachNodePtr, std::set<unsigned int>> reachable_set_to_label{};
        std::unique_ptr<FiniteAutomaton> automaton;

        /// Filter reachable sets that cannot be part of an accepting run of the automaton.
        void _filter_reachable_sets(std::vector<reach::ReachNodePtr> &reachable_sets,
                                                                int step);

        bool _has_accepting_state(const reach::ReachNodePtr &reachable_set);

        std::vector<reach::ReachNodePtr> _split_reachable_set(int step, const reach::ReachNodePtr &reachable_set);

        std::vector<reach::ReachNodePtr> _split_reachable_set_to_minterm(int step, reach::ReachNodePtr reachable_set, Minterm minterm);

    public:
        SemanticSplittingOTFReachableSet(SemanticConfigurationPtr config,
                                         collision::CollisionCheckerPtr collision_checker,
                                         SemanticModelPtr semantic_model,
                                         TrafficRuleInterfacePtr traffic_rule_interface);
    };

    using SemanticSplittingOTFReachableSetPtr = std::shared_ptr<SemanticSplittingOTFReachableSet>;
}
