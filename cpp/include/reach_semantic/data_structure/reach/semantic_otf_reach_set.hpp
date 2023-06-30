#pragma once

#include "reach_semantic/data_structure/reach/semantic_reach_set.hpp"
#include "reach_semantic/data_structure/model_checking/finite_automaton.hpp"

namespace semantic_reach {
    class SemanticOTFReachableSet : public SemanticReachableSet {
    private:
        void _compute_drivable_area_at_step(int const &step) override;

        void _compute_reachable_set_at_step(int const &step) override;

        std::map<int, std::map<std::pair<std::set<unsigned int>, std::set<unsigned int>>, std::vector<reach::ReachPolygonPtr>>> map_step_to_states_to_drivable_area{};
        std::map<int, std::map<std::pair<std::set<unsigned int>, std::set<unsigned int>>, std::vector<reach::ReachNodePtr>>> map_step_to_states_to_propagated_set{};
        std::map<reach::ReachNodePtr, std::set<unsigned int>> reachable_set_to_label{};
        std::unique_ptr<FiniteAutomaton> automaton;

        void _label_reachable_sets_with_automaton_states(std::vector<reach::ReachNodePtr> &reachable_sets,
                                                         bool initial_step = false);

        /// Label the reachable set with the automaton states that are reachable given its propositions.
        void _label_automaton_states(const reach::ReachNodePtr &reachable_set, unsigned int current_state);

        /// Filter reachable sets that cannot be part of an accepting run of the automaton.
        void _filter_reachable_sets(std::vector<reach::ReachNodePtr> &reachable_sets,
                                    int step);

        bool _has_accepting_state(const reach::ReachNodePtr &reachable_set);

    public:
        SemanticOTFReachableSet(SemanticConfigurationPtr config,
                                collision::CollisionCheckerPtr collision_checker,
                                SemanticModelPtr semantic_model,
                                TrafficRuleInterfacePtr traffic_rule_interface);
    };

    using SemanticOTFReachableSetPtr = std::shared_ptr<SemanticOTFReachableSet>;
}
