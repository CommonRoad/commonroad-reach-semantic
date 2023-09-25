#pragma once

#include "reach_semantic/data_structure/reach/semantic_reach_set.hpp"
#include "reach_semantic/data_structure/reach/splitters/minterm_reach_node_splitter.hpp"
#include "reach_semantic/data_structure/model_checking/finite_automaton.hpp"

namespace semantic_reach {
    class SemanticOTFReachableSet : public SemanticReachableSet {
    private:
        std::unique_ptr<FiniteAutomaton> automaton;
        std::unique_ptr<MintermReachNodeSplitter> splitter;

        std::map<int, std::map<std::pair<std::set<unsigned int>, std::set<unsigned int>>, std::vector<reach::ReachPolygonPtr>>> step_to_states_to_drivable_area{};
        std::map<int, std::map<std::pair<std::set<unsigned int>, std::set<unsigned int>>, std::vector<reach::ReachNodePtr>>> step_to_states_to_propagated_set{};

        void _compute_drivable_area_at_step(int const &step) override;

        void _compute_reachable_set_at_step(int const &step) override;

        /**
         * Split the given reachable set along the transitions of the automaton states of its propagation source.
         * For this, consider the outgoing transitions of all automaton states in the labels of the propagation source.
         * We then split and cut the reachable set along the conditions of these transitions.
         *
         * @param step Current step of the reachability analysis.
         * @param reachable_set The reachable set to split.
         * @return List of reachable sets so that each is a subset of the given reachable set, and satisfies the condition of at least one transition (up to overapproximation).
         */
        std::vector<reach::ReachNodePtr> _split_reachable_set(int step, const reach::ReachNodePtr &reachable_set);

        /**
         * Filter reachable sets that cannot be part of an accepting run of the automaton.
         */
        void _filter_reachable_sets(std::vector<reach::ReachNodePtr> &reachable_sets,
                                    int step);

        /**
         * Check if the given reachable set has an accepting state.
         */
        bool _has_accepting_state(const reach::ReachNodePtr &reachable_set);

        /**
         * Deduplicate reachable sets and merge labels of duplicates.
         */
        std::vector<reach::ReachNodePtr>
        _deduplicate_reachable_sets(const std::vector<reach::ReachNodePtr> &reachable_sets);

    public:
        SemanticOTFReachableSet(SemanticConfigurationPtr config,
                                collision::CollisionCheckerPtr collision_checker,
                                SemanticModelPtr semantic_model,
                                TrafficRuleInterfacePtr traffic_rule_interface);

        std::map<reach::ReachNodePtr, std::set<State>> reachable_set_to_label{};
    };
}
