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

        /// Check if the given reachable set has an accepting state.
        bool _has_accepting_state(const reach::ReachNodePtr &reachable_set);

        /// Split the given reachable set along the transitions of the automaton states of its propagation source.
        /// For this, consider the outgoing transitions of all automaton states in the labels of the propagation source.
        /// We then split and cut the reachable set along the conditions of these transitions.
        /// @param step Current step of the reachability analysis.
        /// @param reachable_set The reachable set to split.
        /// @return List of reachable sets so that each is a subset of the given reachable set, and satisfies the condition of at least one transition (up to overapproximation).
        std::vector<reach::ReachNodePtr> _split_reachable_set(int step, const reach::ReachNodePtr &reachable_set);

        /// Split the given reachable sets along the given transitions.
        /// This is a recursive function that splits the reachable sets along the given transitions.
        /// The goal is to reuse as many splits as possible.
        /// First, we select a (not yet finished) literal that we will use for splitting in this step.
        /// Then, we partition the transitions into those that depend on the literal and those that do not.
        /// To further handle the former, we need to restrict the reachable sets to the chosen literal.
        /// The chosen literal is marked as finished for the restricted reachable sets.
        /// Thus, when _split_to_minterms is called, all nodes in reachable_sets satisfy all literals in finished_literals.
        /// We then recursively split the original reachable sets along the transitions that do not depend on the literal,
        /// and the restricted reachable sets along the transitions that do depend on the literal.
        /// The recursion ends, when there are no more reachable sets, because restricting them along the literal resulted in an empty set.
        /// The recursion also ends, when we considered all literals.
        /// In this case, we label the reachable sets with the target states of the transitions that they satisfy.
        /// @param step Current step of the reachability analysis.
        /// @param reachable_sets Reachable sets to split.
        /// @param transitions Transitions to split along.
        /// @param finished_literals Literals that we no longer have to consider.
        /// @param regionized Whether the reachable sets are already split into regions.
        /// @return List of reachable sets so that each is a subset of the given reachable sets, and satisfies the condition of at least one transition (up to overapproximation).
        std::vector<reach::ReachNodePtr>
        _split_to_minterms(int step, const std::vector<reach::ReachNodePtr> &reachable_sets,
                           const std::map<Minterm, std::set<unsigned int>> &transitions,
                           std::set<Literal> &finished_literals, bool regionized);

        /// Restrict the reachable sets to the given literal.
        /// If restricting requires lanelet information, we first split the reachable sets into regions (if we haven't already).
        /// If we clone the reachable sets, we also copy the labels from the original nodes to the clones.
        /// @param step Current step of the reachability analysis.
        /// @param reachable_sets The reachable sets to restrict.
        /// @param literal The literal to restrict to.
        /// @param regionized Whether the reachable sets are already split into regions.
        /// @param clone Whether to clone the reachable sets before restricting.
        /// @return The restricted reachable sets and whether they were split into regions.
        std::pair<std::vector<reach::ReachNodePtr>, bool>
        _restrict_to_literal(int step, const std::vector<reach::ReachNodePtr> &reachable_sets, const Literal &literal,
                             bool regionized, bool clone);

        /// Partition the transitions into those that depend on the literal and those that don't.
        /// @param literal The literal to partition the transitions along.
        /// @param transitions The transitions to partition.
        /// @return A tuple of two dictionaries, the first containing the transitions that don't depend on the literal, the second containing the transitions that do.
        static std::pair<std::map<Minterm, std::set<unsigned int>>, std::map<Minterm, std::set<unsigned int>>>
        _partition_transitions(const Literal &literal, const std::map<Minterm, std::set<unsigned int>> &transitions);

        /// Selects the next literal along which to split the reachable set.
        /// We use a greedy approach, so we choose the literal that occurs most often in the minterms.
        /// @param minterms The list of minterms to consider.
        /// @param ignored_literals These literals will be ignored when choosing the next literal.
        /// @return The literal that occurs most often in minterms.
        static std::optional<Literal>
        _choose_next_literal(const std::vector<Minterm> &minterms, const std::set<Literal> &ignored_literals);

    public:
        SemanticSplittingOTFReachableSet(SemanticConfigurationPtr config,
                                         collision::CollisionCheckerPtr collision_checker,
                                         SemanticModelPtr semantic_model,
                                         TrafficRuleInterfacePtr traffic_rule_interface);
    };

    using SemanticSplittingOTFReachableSetPtr = std::shared_ptr<SemanticSplittingOTFReachableSet>;
}
