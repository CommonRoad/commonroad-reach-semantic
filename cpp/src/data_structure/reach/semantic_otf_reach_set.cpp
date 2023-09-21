#include "reach_semantic/data_structure/reach/semantic_otf_reach_set.hpp"
#include "reach_semantic/data_structure/reach/predicates/predicate.hpp"
#include "reach_semantic/utility/environment_model.hpp"
#include "reach_semantic/utility/reach_operation.hpp"

#include "reachset/utility/reach_operation.hpp"

#include <commonroad_cpp/interfaces/commonroad/input_utils.h>

#include <utility>

using namespace semantic_reach;

SemanticOTFReachableSet::SemanticOTFReachableSet(semantic_reach::SemanticConfigurationPtr config,
                                                 collision::CollisionCheckerPtr collision_checker,
                                                 semantic_reach::SemanticModelPtr semantic_model,
                                                 semantic_reach::TrafficRuleInterfacePtr traffic_rule_interface)
        : SemanticReachableSet(std::move(config), std::move(collision_checker), std::move(semantic_model),
                               std::move(traffic_rule_interface)) {
    _initialize_zero_state_polygons();

    // Construct finite automaton from traffic rules
    automaton = std::make_unique<FiniteAutomaton>(rule_interface->vec_specifications_ltl,
                                                  this->config->config_traffic_rule.mode_automata);

    // Create environment model
    const auto &[obstacles, roadNetwork, scenario_dt] = InputUtils::getDataFromCommonRoad(
            this->config->config_general.path_scenarios + this->config->config_general.name_scenario + ".xml");
    resample_obstacle_states(obstacles, scenario_dt, this->config->config_planning.dt);
    world = std::make_shared<World>(step_start, roadNetwork, std::vector<std::shared_ptr<Obstacle>>{}, obstacles,
                                    this->config->config_planning.dt);

    auto config_ccs{this->config->config_planning.CLCS};
    ego_ccs = std::make_shared<geometry::CurvilinearCoordinateSystem>(config_ccs->referencePathOriginal());
    // FIXME: Use projection domain and epsilons from config_ccs (using defaults for now)
    // Currently, these have weird values
    // Maybe this is a consequence of config_ccs being initialized as CCS of drivability checker, while we only link against the environment model
//    ego_ccs = std::make_shared<geometry::CurvilinearCoordinateSystem>(config_ccs->referencePathOriginal(),
//                                                                      config_ccs->defaultProjectionDomainLimit(),
//                                                                      config_ccs->eps(), config_ccs->eps2());


    // Compute initial reachable set
    SemanticOTFReachableSet::_compute_drivable_area_at_step(step_start);
    SemanticOTFReachableSet::_compute_reachable_set_at_step(step_start);
    _vec_steps_computed.emplace_back(step_start);
}

void SemanticOTFReachableSet::_compute_drivable_area_at_step(const int &step) {
    std::vector<ReachNodePtr> propagated_sets;
    if (step != step_start) {
        auto reachable_set_previous = map_step_to_reachable_set[step - 1];
        if (reachable_set_previous.empty()) {
            map_step_to_drivable_area[step] = {};
            map_step_to_propagated_set[step] = {};
            return;
        }

        propagated_sets = _propagate_reachable_set(reachable_set_previous);
    } else {
        propagated_sets = _construct_initial_reachable_sets();
    }

    std::vector<reach::ReachNodePtr> propagated_sets_split{};
    for (const auto &propagated_set: propagated_sets) {
        auto split_reachable_sets = _split_reachable_set(step, propagated_set);
        propagated_sets_split.insert(propagated_sets_split.end(), split_reachable_sets.begin(),
                                     split_reachable_sets.end());
    }

    // partition propagated sets by their automaton states
    std::map<std::pair<std::set<unsigned int>, std::set<unsigned int>>, std::vector<reach::ReachNodePtr>> map_states_to_propagated_set{};
    for (auto const &propagated_set: propagated_sets_split) {
        std::pair<std::set<unsigned int>, std::set<unsigned int>> key;
        if (step != step_start) {
            key = std::make_pair(reachable_set_to_label[propagated_set->vec_nodes_source[0]],
                                 reachable_set_to_label[propagated_set]);
        } else {
            key = std::make_pair(std::set<unsigned int>{automaton->initial_state()},
                                 reachable_set_to_label[propagated_set]);
        }
        map_states_to_propagated_set[key].emplace_back(propagated_set);
    }

    // merge, collision check, and repartition propagated sets
    // this is done individually for each group calculated above, because we must not merge sets semantically different base sets
    // it is necessary to also consider the states of the propagation source for the partitioning, because only if these are equal, the automaton cannot distinguish the base sets
    // if only the target states were considered, the automaton could possibly distinguish them if the source states reach the target state via different propositions
    std::map<std::pair<std::set<unsigned int>, std::set<unsigned int>>, std::vector<reach::ReachPolygonPtr>> map_states_to_drivable_area{};
    std::vector<reach::ReachPolygonPtr> vec_drivable_area{};
    for (const auto &[states, propagated_sets_per_states]: map_states_to_propagated_set) {
        auto vec_rectangles_projected = reach::project_base_sets_to_position_domain(propagated_sets_per_states);
        auto drivable_area_at_states = _collision_check_and_repartition(vec_rectangles_projected, step);
        map_states_to_drivable_area[states] = drivable_area_at_states;
        vec_drivable_area.insert(vec_drivable_area.end(), drivable_area_at_states.begin(),
                                 drivable_area_at_states.end());
    }

    map_step_to_drivable_area[step] = vec_drivable_area;
    map_step_to_states_to_drivable_area[step] = map_states_to_drivable_area;
    map_step_to_states_to_propagated_set[step] = map_states_to_propagated_set;
    map_step_to_propagated_set[step] = propagated_sets_split;
}

void SemanticOTFReachableSet::_compute_reachable_set_at_step(const int &step) {
    auto map_states_to_propagated_set = map_step_to_states_to_propagated_set[step];
    auto map_states_to_drivable_area = map_step_to_states_to_drivable_area[step];

    if (map_states_to_drivable_area.empty()) {
        map_step_to_reachable_set[step] = {};
        return;
    }

    auto num_threads = config->reachable_set().num_threads;

    // discard drivable area with small area if there are more than one node (this is subject to change)
    unsigned long num_drivable_area = 0;
    for (auto const &[proposition_holder, drivable_area]: map_states_to_drivable_area) {
        num_drivable_area += drivable_area.size();
    }
    bool discard_small_node = (num_drivable_area > 1) && config->reachable_set().discard_small_nodes;

    // work with the reachable sets partitioned by propositions here, because otherwise it could happen
    // that we merge two reachable sets with different propositions when they intersect with the same drivable area
    vector<reach::ReachNodePtr> new_reachable_sets{};
    for (auto const &[automaton_states, drivable_area]: map_states_to_drivable_area) {
        auto propagated_sets = map_states_to_propagated_set[automaton_states];

        auto reachable_sets = reach::construct_reach_nodes(drivable_area, propagated_sets, num_threads);

        if (discard_small_node) {
            reachable_sets = semantic_reach::discard_nodes_with_short_edge(reachable_sets,
                                                                           config->reachable_set().length_edge_node_min);
        }

        if (step != step_start) {
            // this sets the correct step for the new reach nodes ...
            reachable_sets = reach::connect_children_to_parents(step, reachable_sets, num_threads);
        } else {
            // ... so we need to do this manually for the initial step, as there are no parents here
            for (auto &node: reachable_sets) {
                node->step = step;
            }
        }


        // assign label to all newly constructed reach nodes
        auto [_, target_states] = automaton_states;
        for (const auto &node: reachable_sets) {
            reachable_set_to_label[node] = target_states;
        }
        new_reachable_sets.insert(new_reachable_sets.end(),
                                  std::make_move_iterator(reachable_sets.begin()),
                                  std::make_move_iterator(reachable_sets.end()));
    }
    map_step_to_reachable_set[step] = new_reachable_sets;
}

std::vector<reach::ReachNodePtr>
SemanticOTFReachableSet::_split_reachable_set(int step, const reach::ReachNodePtr &reachable_set) {
    auto current_states =
            step == step_start ? std::set<unsigned int>{automaton->initial_state()}
                               : reachable_set_to_label[reachable_set->vec_nodes_source[0]];
    auto transitions = automaton->combined_transitions_from(current_states);
    std::set<Literal> finished_literals{};
    auto constrained_reachable_sets = _split_to_minterms(step, {reachable_set}, transitions, finished_literals, false);

    _filter_reachable_sets(constrained_reachable_sets, step);

    constrained_reachable_sets = _deduplicate_reachable_sets(constrained_reachable_sets);

    return constrained_reachable_sets;
}

void
SemanticOTFReachableSet::_filter_reachable_sets(std::vector<reach::ReachNodePtr> &reachable_sets, int step) {
    bool is_final_step = (step == step_end);
    reachable_sets.erase(std::remove_if(reachable_sets.begin(), reachable_sets.end(),
                                        [this, is_final_step](const reach::ReachNodePtr &node) {
                                            return this->reachable_set_to_label[node].empty() ||
                                                   (is_final_step && !_has_accepting_state(node));
                                        }), reachable_sets.end());
}

bool SemanticOTFReachableSet::_has_accepting_state(const reach::ReachNodePtr &reachable_set) {
    auto states = reachable_set_to_label[reachable_set];
    return std::any_of(states.begin(), states.end(), [this](const int &state) {
        return automaton->is_accepting_state(state);
    });
}

std::vector<reach::ReachNodePtr>
SemanticOTFReachableSet::_deduplicate_reachable_sets(const std::vector<reach::ReachNodePtr> &reachable_sets) {
    std::vector<reach::ReachNodePtr> unique_reachable_sets;
    for (const auto &reachable_set: reachable_sets) {
        bool is_duplicate = false;
        for (const auto &other: unique_reachable_sets) {
            bool equal_lon = reachable_set->polygon_lon->vec_vertices == other->polygon_lon->vec_vertices;
            bool equal_lat = reachable_set->polygon_lat->vec_vertices == other->polygon_lat->vec_vertices;
            if (equal_lon && equal_lat) {
                auto &my_labels = reachable_set_to_label[reachable_set];
                reachable_set_to_label[other].insert(my_labels.begin(), my_labels.end());
                is_duplicate = true;
                break;
            }
        }
        if (!is_duplicate) {
            unique_reachable_sets.push_back(reachable_set);
        }
    }
    return unique_reachable_sets;
}

std::vector<reach::ReachNodePtr>
SemanticOTFReachableSet::_split_to_minterms(int step, const std::vector<reach::ReachNodePtr> &reachable_sets,
                                            const std::vector<std::pair<Minterm, unsigned int>> &transitions,
                                            std::set<Literal> &finished_literals, bool regionized) {
    if (reachable_sets.empty() || transitions.empty()) {
        // if there are no reachable sets or no transitions, there is nothing to split
        return reachable_sets;
    }

    // select the next literal to split on
    std::vector<Minterm> minterms;
    minterms.reserve(transitions.size());
    std::transform(transitions.begin(), transitions.end(), std::back_inserter(minterms),
                   [](const std::pair<Minterm, unsigned int> &transition) {
                       return transition.first;
                   });
    auto literal_to_split_opt = _choose_next_literal(minterms, finished_literals);

    // BASE CASE: if there is no literal to split, we are done
    if (!literal_to_split_opt) {
        // all nodes in reachable_sets satisfy the transition condition, so label them with the destination states
        std::set<unsigned int> state_labels;
        std::transform(transitions.begin(), transitions.end(), std::inserter(state_labels, state_labels.end()),
                       [](const std::pair<Minterm, unsigned int> &transition) {
                           return transition.second;
                       });
        for (const auto &reachable_set: reachable_sets) {
            reachable_set_to_label[reachable_set] = state_labels;
        }
        return reachable_sets;
    }
    auto literal_to_split = literal_to_split_opt.value();

    // partition the transitions into those whose label needs the literal and those that don't
    auto [not_needs_literal, needs_literal] = _partition_transitions(literal_to_split, transitions);

    // if there are transitions that don't need the current literal we have to clone the reach nodes before restricting
    // so that we can keep the original nodes for those transitions
    auto [restricted_reachable_sets, restriction_regionized] = _restrict_to_literal(step, reachable_sets,
                                                                                    literal_to_split, regionized,
                                                                                    !not_needs_literal.empty());

    // recurse to split along the remaining literals
    if (!not_needs_literal.empty()) {
        std::set<Literal> finished_literals_restricted{finished_literals};
        finished_literals_restricted.insert(literal_to_split);
        // note that only the restricted reachable sets might have been regionized
        auto restricted_reachable_sets_not_needs_literal = _split_to_minterms(step, reachable_sets, not_needs_literal,
                                                                              finished_literals, regionized);
        auto restricted_reachable_sets_needs_literal = _split_to_minterms(step, restricted_reachable_sets,
                                                                          needs_literal, finished_literals_restricted,
                                                                          regionized || restriction_regionized);
        restricted_reachable_sets_not_needs_literal.insert(restricted_reachable_sets_not_needs_literal.end(),
                                                           std::make_move_iterator(
                                                                   restricted_reachable_sets_needs_literal.begin()),
                                                           std::make_move_iterator(
                                                                   restricted_reachable_sets_needs_literal.end()));
        return restricted_reachable_sets_not_needs_literal;
    } else {
        finished_literals.insert(literal_to_split);
        return _split_to_minterms(step, restricted_reachable_sets, needs_literal, finished_literals,
                                  regionized || restriction_regionized);
    }
}

std::pair<std::vector<reach::ReachNodePtr>, bool>
SemanticOTFReachableSet::_restrict_to_literal(int step, const std::vector<reach::ReachNodePtr> &reachable_sets,
                                              const semantic_reach::Literal &literal, bool regionized,
                                              bool clone) {
    std::vector<reach::ReachNodePtr> to_restrict;
    if (clone) {
        to_restrict.reserve(reachable_sets.size());
        std::transform(reachable_sets.begin(), reachable_sets.end(), std::back_inserter(to_restrict),
                       [](const reach::ReachNodePtr &node) {
                           return node->clone();
                       });
        // if we already split to regions, we need to copy labels from the original nodes to the clones
        for (std::pair it{reachable_sets.begin(), to_restrict.begin()};
             it.first != reachable_sets.end(); ++it.first, ++it.second) {
            labeler->copy_labels(*it.first, {*it.second});
        }
    } else {
        to_restrict = reachable_sets;
    }

    auto pred = Predicate::from_proposition(literal.first, literal.second);
    if (pred->needs_lanelets && !regionized) {
        // if the predicate needs lanelets, we need to split the reachable sets into regions first (if we haven't already)
        to_restrict = labeler->split_wrt_regions(step, to_restrict);
    }
    // restrict the reachable sets to the predicate
    std::vector<reach::ReachNodePtr> restricted_reachable_sets{};
    for (const auto &node: to_restrict) {
        auto restricted_nodes = pred->needs_lanelets ?
                                pred->restrict_reach_node(step, node, semantic_model, world, ego_ccs,
                                                          labeler->reachable_set_to_lanelet_ids[node]) :
                                pred->restrict_reach_node(step, node, semantic_model, world, ego_ccs);
        restricted_reachable_sets.insert(restricted_reachable_sets.end(),
                                         std::make_move_iterator(restricted_nodes.begin()),
                                         std::make_move_iterator(restricted_nodes.end()));
    }
    return {restricted_reachable_sets, pred->needs_lanelets};
}

std::pair<std::vector<std::pair<Minterm, unsigned int>>, std::vector<std::pair<Minterm, unsigned int>>>
SemanticOTFReachableSet::_partition_transitions(const semantic_reach::Literal &literal,
                                                const std::vector<std::pair<Minterm, unsigned int>> &transitions) {
    std::vector<std::pair<Minterm, unsigned int>> not_needs_literal{};
    std::vector<std::pair<Minterm, unsigned int>> needs_literal{};

    std::partition_copy(transitions.begin(), transitions.end(), std::back_inserter(needs_literal),
                        std::back_inserter(not_needs_literal),
                        [&](const auto &transition) {
                            return std::count(transition.first.begin(), transition.first.end(), literal) > 0;
                        });

    return {not_needs_literal, needs_literal};
}

std::optional<Literal> SemanticOTFReachableSet::_choose_next_literal(const std::vector<Minterm> &minterms,
                                                                     const std::set<Literal> &ignored_literals) {
    // remove duplicates so that we do not make a minterm more important if it leads to multiple states
    // TODO: does this make sense?
    std::set<Minterm> unique_minterms{minterms.begin(), minterms.end()};

    std::map<Literal, int> literal_counts{};
    for (const auto &minterm: unique_minterms) {
        for (const auto &literal: minterm) {
            if (ignored_literals.find(literal) == ignored_literals.end()) {
                literal_counts[literal]++;
            }
        }
    }

    // the literals that occur most often are candidates for the next literal
    int max_cnt = std::max_element(literal_counts.begin(), literal_counts.end(),
                                   [](const std::pair<Literal, int> &a, const std::pair<Literal, int> &b) {
                                       return a.second < b.second;
                                   })->second;
    std::vector<Literal> candidates;
    for (const auto &[literal, cnt]: literal_counts) {
        if (cnt == max_cnt) {
            candidates.push_back(literal);
        }
    }

    // prefer predicates that don't need lanelets, as this avoids splitting to regions
    // TODO: we could choose a different ordering here or make this configurable
    std::sort(candidates.begin(), candidates.end(), [](const Literal &a, const Literal &b) {
        return !Predicate::from_proposition(a.first, a.second)->needs_lanelets &&
               Predicate::from_proposition(b.first, b.second)->needs_lanelets;
    });

    return candidates.empty() ? std::nullopt : std::optional<Literal>{candidates[0]};
}
