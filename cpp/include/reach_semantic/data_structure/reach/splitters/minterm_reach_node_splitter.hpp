#pragma once

#include "reach_semantic/data_structure/model_checking/finite_automaton.hpp"
#include "reach_semantic/data_structure/reach/reachable_set_labeler.hpp"
#include "reach_semantic/data_structure/reach/splitters/region_reach_node_splitter.hpp"
#include "reach_semantic/data_structure/semantic_model.hpp"

#include "reachset/data_structure/reach/reach_node.hpp"

#include <map>
#include <set>

namespace semantic_reach {
/**
 * Split reach nodes according to given minterms.
 */
class MintermReachNodeSplitter {
  private:
    SemanticModelPtr semantic_model;
    std::unique_ptr<RegionReachNodeSplitter> region_splitter;
    std::map<reach::ReachNodePtr, std::set<int>> node_to_lanelet_ids;

    /**
     * Implementation of split_to_minterms.
     *
     * This is a recursive function that splits the reachable sets along the given minterms.
     * The goal is to reuse as many splits as possible.
     * First, we select a (not yet finished) literal that we will use for splitting in this step.
     * Then, we partition the minterms into those that depend on the literal and those that do not.
     * To further handle the former, we need to restrict the reachable sets to the chosen literal.
     * The chosen literal is marked as finished for the restricted reachable sets.
     * Thus, when _split_to_minterms is called, all nodes in reachable_sets satisfy all literals in finished_literals.
     * We then recursively split the original reachable sets along the minterms that do not depend on the literal,
     * and the restricted reachable sets along the minterms that do depend on the literal.
     * The recursion ends, when there are no more reachable sets, because restricting them along the literal resulted in
     * an empty set. The recursion also ends, when we considered all literals. In this case, we label the reachable sets
     * with the target states of the minterms that they satisfy.
     *
     * @param step Current step of the reachability analysis.
     * @param reachable_sets Reachable sets to split.
     * @param minterms Minterms to split along.
     * @param finished_literals Literals that we no longer have to consider.
     * @param regionized Whether the reachable sets are already split into regions.
     * @param result List to write the result to.
     */
    void _split_to_minterms(int step, const std::vector<reach::ReachNodePtr> &reachable_sets,
                            const std::set<Minterm> &minterms, std::set<Literal> &finished_literals, bool regionized,
                            std::vector<std::pair<Minterm, std::vector<reach::ReachNodePtr>>> &result);

    /**
     * Restrict the reachable sets to the given literal.
     *
     * If restricting requires lanelet information, we first split the reachable sets into regions.
     * If we clone the reachable sets, we also copy the labels from the original nodes to the clones.
     *
     * @param step Current step of the reachability analysis.
     * @param reachable_sets The reachable sets to restrict.
     * @param literal The literal to restrict to.
     * @param regionized Whether the reachable sets are already split into regions.
     * @param clone Whether to clone the reachable sets before restricting.
     * @return The restricted reachable sets and whether they were split into regions.
     */
    std::pair<std::vector<reach::ReachNodePtr>, bool>
    _restrict_to_literal(int step, const std::vector<reach::ReachNodePtr> &reachable_sets, const Literal &literal,
                         bool regionized, bool clone);

    /**
     * Partition the minterms into those that contain the literal and those that don't.
     *
     * @param literal The literal to partition the minterms along.
     * @param minterms The minterms to partition.
     * @return A tuple of two minterm lists, the first containing the minterms don't contain the literal,
     *     the second containing the minterms that do.
     */
    static std::pair<std::set<Minterm>, std::set<Minterm>> _partition_minterms(const Literal &literal,
                                                                               const std::set<Minterm> &minterms);

    /**
     * Selects the next literal along which to split the reachable set.
     *
     * We use a greedy approach, so we choose the literal that occurs most often in the minterms.
     *
     * @param minterms The list of minterms to consider.
     * @param ignored_literals These literals will be ignored when choosing the next literal.
     * @return The literal that occurs most often in minterms.
     */
    static std::optional<Literal> _choose_next_literal(const std::set<Minterm> &minterms,
                                                       const std::set<Literal> &ignored_literals);

  public:
    /**
     * Create a new minterm reach node splitter.
     *
     * @param semantic_model The semantic model to use for splitting.
     */
    explicit MintermReachNodeSplitter(SemanticModelPtr semantic_model);

    /**
     * Split the given reachable sets along the given minterms.
     *
     * @param step Current step of the reachability analysis.
     * @param reachable_set Reachable set to split.
     * @param minterms Minterms to split along.
     * @return List of reachable sets for each given minterm.
     *     The union of the reachable sets associated with a minterm overapproximates the original reachable set
     *     restricted to states that satisfy the minterm.
     *     A minterm may be omitted if the corresponding restricted set is empty.
     */
    std::vector<std::pair<Minterm, std::vector<reach::ReachNodePtr>>>
    split_to_minterms(int step, const reach::ReachNodePtr &reachable_set, const std::set<Minterm> &minterms);
};
} // namespace semantic_reach
