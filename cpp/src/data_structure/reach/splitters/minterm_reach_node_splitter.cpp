#include "reach_semantic/data_structure/reach/splitters/minterm_reach_node_splitter.hpp"
#include "reach_semantic/data_structure/reach/predicates/predicate.hpp"
#include "reach_semantic/utility/environment_model.hpp"

#include <commonroad_cpp/interfaces/commonroad/input_utils.h>

using namespace semantic_reach;

MintermReachNodeSplitter::MintermReachNodeSplitter(SemanticModelPtr semantic_model,
                                                   const SemanticConfigurationPtr &config)
    : semantic_model(std::move(semantic_model)),
      region_splitter(std::make_unique<RegionReachNodeSplitter>(this->semantic_model)),
      predicate_factory(PredicateFactory{std::make_unique<PredicateConfiguration>(*config)}) {
    // Create environment model
    const auto &[obstacles, roadNetwork, scenario_dt] = InputUtils::getDataFromCommonRoad(
        config->config_general.path_scenarios + config->config_general.name_scenario + ".xml");
    resample_obstacle_states(obstacles, scenario_dt, config->config_planning.dt);
    world = std::make_shared<World>(config->config_planning.step_start, roadNetwork,
                                    std::vector<std::shared_ptr<Obstacle>>{}, obstacles, config->config_planning.dt);

    auto config_ccs{config->config_planning.CLCS};
    ego_ccs = std::make_shared<geometry::CurvilinearCoordinateSystem>(config_ccs->referencePathOriginal(),
                                                                      config_ccs->defaultProjectionDomainLimit(),
                                                                      config_ccs->eps(), config_ccs->eps2());
}

std::vector<std::pair<Minterm, std::vector<reach::ReachNodePtr>>>
MintermReachNodeSplitter::split_to_minterms(int step, const reach::ReachNodePtr &reachable_set,
                                            const std::vector<Minterm> &minterms) {
    std::vector<std::pair<Minterm, std::vector<reach::ReachNodePtr>>> result{};
    LiteralSet finished_literals{};
    _split_to_minterms(step, {reachable_set}, minterms, finished_literals, false, result);
    return result;
}

void MintermReachNodeSplitter::_split_to_minterms(
    int step, const std::vector<reach::ReachNodePtr> &reachable_sets, const std::vector<Minterm> &minterms,
    const LiteralSet &finished_literals, bool regionized,
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

    auto pred = predicate_factory.predicate_from_proposition(literal.first, literal.second);

    // if the predicate needs lanelets, we need to split the reachable sets into regions first (if we haven't already)
    if (pred->needs_lanelets && !regionized) {
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
        auto restricted_nodes =
            pred->needs_lanelets
                ? pred->restrict_reach_node(step, node, semantic_model, world, ego_ccs, node_to_lanelet_ids[node])
                : pred->restrict_reach_node(step, node, semantic_model, world, ego_ccs);
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

std::pair<std::vector<Minterm>, std::vector<Minterm>>
MintermReachNodeSplitter::_partition_minterms(const Literal &literal, const std::vector<Minterm> &minterms) {
    std::vector<Minterm> not_needs_literal{};
    std::vector<Minterm> needs_literal{};

    std::partition_copy(minterms.begin(), minterms.end(), std::inserter(needs_literal, needs_literal.begin()),
                        std::inserter(not_needs_literal, not_needs_literal.begin()),
                        [&](const auto &minterm) { return std::count(minterm.begin(), minterm.end(), literal) > 0; });

    return {not_needs_literal, needs_literal};
}

std::optional<Literal> MintermReachNodeSplitter::_choose_next_literal(const std::vector<Minterm> &minterms,
                                                                      const LiteralSet &ignored_literals) const {
    std::unordered_map<Literal, int> literal_counts;
    for (const auto &minterm : minterms) {
        for (const auto &literal : minterm) {
            ++literal_counts[literal];
        }
    }

    // remove ignored literals
    // Removing ignored literals after counting is more efficient than not counting them in the first place,
    // as for this we would need to do a set lookup in each iteration of the inner loop above
    for (const auto &ignored_literal : ignored_literals) {
        literal_counts.erase(ignored_literal);
    }

    // TODO: we could choose a different ordering here or make this configurable
    auto best_candidate = std::max_element(
        literal_counts.begin(), literal_counts.end(),
        [this](const std::pair<Literal, int> &count1, const std::pair<Literal, int> &count2) {
            if (count1.second < count2.second) {
                // the literals that occur most often are the best candidates for the next literal
                return true;
            } else if (count1.second == count2.second) {
                // if the literals occur equally often, we prefer literals that don't need lanelets,
                // as this avoids splitting to regions early
                return !predicate_factory.predicate_from_proposition(count2.first.first, count2.first.second)
                            ->needs_lanelets;
            } else {
                return false;
            }
        });

    return best_candidate == literal_counts.end() ? std::nullopt : std::optional<Literal>{best_candidate->first};
}
