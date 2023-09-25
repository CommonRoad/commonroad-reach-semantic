#include "reach_semantic/data_structure/reach/splitters/minterm_reach_node_splitter.hpp"
#include "reach_semantic/data_structure/reach/predicates/predicate.hpp"

using namespace semantic_reach;

MintermReachNodeSplitter::MintermReachNodeSplitter(SemanticModelPtr semantic_model)
    : semantic_model(std::move(semantic_model)),
      region_splitter(std::make_unique<RegionReachNodeSplitter>(this->semantic_model)) {}

std::vector<std::pair<Minterm, std::vector<reach::ReachNodePtr>>>
MintermReachNodeSplitter::split_to_minterms(int step, const reach::ReachNodePtr &reachable_set,
                                            const std::set<Minterm> &minterms) {
    std::vector<std::pair<Minterm, std::vector<reach::ReachNodePtr>>> result{};
    std::set<Literal> finished_literals{};
    _split_to_minterms(step, {reachable_set}, minterms, finished_literals, false, result);
    return result;
}

void MintermReachNodeSplitter::_split_to_minterms(
    int step, const std::vector<reach::ReachNodePtr> &reachable_sets, const std::set<Minterm> &minterms,
    std::set<Literal> &finished_literals, bool regionized,
    std::vector<std::pair<Minterm, std::vector<reach::ReachNodePtr>>> &result) {
    // BASE CASE: if there are no reachable sets or no minterms, we are done
    if (reachable_sets.empty() || minterms.empty()) {
        return;
    }

    // select the next literal to split on
    auto literal_to_split_opt = _choose_next_literal(minterms, finished_literals);

    // BASE CASE: if there is no literal to split, we are done
    if (!literal_to_split_opt.has_value()) {
        // if we did not find a literal to split, all remaining minterms are the same.
        assert(minterms.size() == 1);
        // moreover, the literals in the remaining minterms are all finished.
        assert(*minterms.begin() == finished_literals);
        // write the reachable sets to the result
        result.emplace_back(finished_literals, reachable_sets);
        return;
    }
    auto literal_to_split = literal_to_split_opt.value();

    // partition the minterms into those that contain the literal and those that don't
    auto [not_has_literal, has_literal] = _partition_minterms(literal_to_split, minterms);

    // if there are minterms that don't contain the current literal, we have to clone the reach nodes before restricting
    // so that we can keep the original nodes for those minterms
    auto [restricted_reachable_sets, restriction_regionized] =
        _restrict_to_literal(step, reachable_sets, literal_to_split, regionized, !not_has_literal.empty());

    // recurse to split along the remaining literals
    // note that only the restricted reachable sets might have been regionized
    _split_to_minterms(step, reachable_sets, not_has_literal, finished_literals, regionized, result);
    // we need to copy the set here to not interfere with subsequent calls
    auto finished_literals_restricted{finished_literals};
    finished_literals_restricted.insert(literal_to_split);
    _split_to_minterms(step, restricted_reachable_sets, has_literal, finished_literals_restricted,
                       regionized || restriction_regionized, result);
}

std::pair<std::vector<reach::ReachNodePtr>, bool>
MintermReachNodeSplitter::_restrict_to_literal(int step, const std::vector<reach::ReachNodePtr> &reachable_sets,
                                               const Literal &literal, bool regionized, bool clone) {
    std::vector<reach::ReachNodePtr> to_restrict;
    if (clone) {
        to_restrict.reserve(reachable_sets.size());
        std::transform(reachable_sets.begin(), reachable_sets.end(), std::back_inserter(to_restrict),
                       [](const reach::ReachNodePtr &node) { return node->clone(); });
        // if we already split to regions, we need to copy the lanelet ids from the original nodes to the clones
        for (std::pair it{reachable_sets.begin(), to_restrict.begin()}; it.first != reachable_sets.end();
             ++it.first, ++it.second) {
            node_to_lanelet_ids[*it.second] = node_to_lanelet_ids[*it.first];
        }
    } else {
        to_restrict = reachable_sets;
    }

    Predicate pred = Predicate::from_proposition(literal.first, literal.second);

    // if the predicate needs lanelets, we need to split the reachable sets into regions first (if we haven't already)
    if (pred.needs_lanelets && !regionized) {
        std::vector<reach::ReachNodePtr> regionized_reachable_sets{};
        for (const auto &node : to_restrict) {
            for (const auto &[region, restricted_node] : region_splitter->split_wrt_regions(node)) {
                node_to_lanelet_ids[restricted_node] = region->set_ids_lanelets;
                regionized_reachable_sets.emplace_back(restricted_node);
            }
        }
        to_restrict = std::move(regionized_reachable_sets);
        regionized = true;
    }

    std::vector<reach::ReachNodePtr> restricted_reachable_sets{};
    for (const auto &node : to_restrict) {

        // restrict the reachable sets to the predicate
        auto restricted_nodes = pred.needs_lanelets
                                    ? pred.restrict_reach_node(step, node, semantic_model, node_to_lanelet_ids[node])
                                    : pred.restrict_reach_node(step, node, semantic_model);
        // restricting might create clones, so we need to copy the lanelet ids again
        if (regionized) {
            for (const auto &restricted_node : restricted_nodes) {
                node_to_lanelet_ids[restricted_node] = node_to_lanelet_ids.at(node);
            }
        }
        restricted_reachable_sets.insert(restricted_reachable_sets.end(),
                                         std::make_move_iterator(restricted_nodes.begin()),
                                         std::make_move_iterator(restricted_nodes.end()));
    }

    return {restricted_reachable_sets, regionized};
}

std::pair<std::set<Minterm>, std::set<Minterm>>
MintermReachNodeSplitter::_partition_minterms(const Literal &literal, const std::set<Minterm> &minterms) {
    std::set<Minterm> not_needs_literal{};
    std::set<Minterm> needs_literal{};

    std::partition_copy(minterms.begin(), minterms.end(), std::inserter(needs_literal, needs_literal.begin()),
                        std::inserter(not_needs_literal, not_needs_literal.begin()),
                        [&](const auto &minterm) { return std::count(minterm.begin(), minterm.end(), literal) > 0; });

    return {not_needs_literal, needs_literal};
}

std::optional<Literal> MintermReachNodeSplitter::_choose_next_literal(const std::set<Minterm> &minterms,
                                                                      const std::set<Literal> &ignored_literals) {
    std::map<Literal, int> literal_counts{};
    for (const auto &minterm : minterms) {
        for (const auto &literal : minterm) {
            if (ignored_literals.find(literal) == ignored_literals.end()) {
                literal_counts[literal]++;
            }
        }
    }

    // the literals that occur most often are candidates for the next literal
    int max_cnt = std::max_element(literal_counts.begin(), literal_counts.end(),
                                   [](const std::pair<Literal, int> &a, const std::pair<Literal, int> &b) {
                                       return a.second < b.second;
                                   })
                      ->second;
    std::vector<Literal> candidates;
    for (const auto &[literal, cnt] : literal_counts) {
        if (cnt == max_cnt) {
            candidates.push_back(literal);
        }
    }

    // prefer predicates that don't need lanelets, as this avoids splitting to regions
    // TODO: we could choose a different ordering here or make this configurable
    std::sort(candidates.begin(), candidates.end(), [](const Literal &a, const Literal &b) {
        return !Predicate::from_proposition(a.first, a.second).needs_lanelets &&
               Predicate::from_proposition(b.first, b.second).needs_lanelets;
    });

    return candidates.empty() ? std::nullopt : std::optional<Literal>{candidates[0]};
}
