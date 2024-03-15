#pragma once

#include "reach_semantic/data_structure/model_checking/finite_automaton.hpp"
#include "reach_semantic/data_structure/reach/predicates/predicate_factory.hpp"
#include "reach_semantic/data_structure/reach/semantic_reach_set.hpp"
#include "reach_semantic/data_structure/reach/splitters/minterm_reach_node_splitter.hpp"

namespace std {
template <> struct hash<std::pair<semantic_reach::StateSet, semantic_reach::StateSet>> {
    size_t operator()(const std::pair<semantic_reach::StateSet, semantic_reach::StateSet> &state_set_pair) const {
        size_t seed = state_set_pair.first.size() + state_set_pair.second.size();
        for (const auto &state : state_set_pair.first) {
            seed ^= boost::hash_value(state);
        }
        for (const auto &state : state_set_pair.second) {
            seed ^= boost::hash_value(state);
        }
        return seed;
    }
};
} // namespace std

namespace semantic_reach {
class SemanticOTFReachableSet : public SemanticReachableSet {
  private:
    std::unique_ptr<FiniteAutomaton> automaton;
    std::unique_ptr<MintermReachNodeSplitter> splitter;

    std::unordered_map<int, std::unordered_map<std::pair<StateSet, StateSet>, std::vector<reach::ReachPolygonPtr>>>
        step_to_states_to_drivable_area{};
    std::unordered_map<int, std::unordered_map<std::pair<StateSet, StateSet>, std::vector<reach::ReachNodePtr>>>
        step_to_states_to_propagated_set{};

    void _compute_drivable_area_at_step(int const &step) override;

    void _compute_reachable_set_at_step(int const &step) override;

    /**
     * Split the given reachable set along the transitions of the automaton states of its propagation source.
     * For this, consider the outgoing transitions of all automaton states in the labels of the propagation source.
     * We then split and cut the reachable set along the conditions of these transitions.
     *
     * @param step Current step of the reachability analysis.
     * @param reachable_set The reachable set to split.
     * @return List of reachable sets so that each is a subset of the given reachable set, and satisfies the condition
     * of at least one transition (up to overapproximation).
     */
    std::vector<reach::ReachNodePtr> _split_reachable_set(int step, const reach::ReachNodePtr &reachable_set);

    /**
     * Filter reachable sets that cannot be part of an accepting run of the automaton.
     *
     * This means they are labeled with at least one state.
     * In the final step, we also require the reachable sets to have at least one accepting state.
     *
     * @param reachable_sets List of reachable sets to filter.
     * @param step Current step of the reachability analysis.
     * @returns List of reachable sets that can be part of an accepting run of the automaton.
     */
    void _filter_reachable_sets(std::vector<reach::ReachNodePtr> &reachable_sets, int step);

    /**
     * Check if the given reachable set has an accepting state.
     *
     * @param reachable_set The reachable set to check.
     * @returns True if and only if the reachable set is labeled with at least one accepting state.
     */
    bool _has_accepting_state(const reach::ReachNodePtr &reachable_set);

    /**
     * Deduplicate reachable sets and merge labels of duplicates.
     *
     * @param reachable_sets List of reachable sets with possible duplicates.
     * @returns List of reachable sets without duplicates.
     */
    std::vector<reach::ReachNodePtr>
    _deduplicate_reachable_sets(const std::vector<reach::ReachNodePtr> &reachable_sets);

  public:
    SemanticOTFReachableSet(SemanticConfigurationPtr config, collision::CollisionCheckerPtr collision_checker,
                            SemanticModelPtr semantic_model, TrafficRuleInterfacePtr traffic_rule_interface);

    std::map<reach::ReachNodePtr, StateSet> reachable_set_to_label{};
};
} // namespace semantic_reach
