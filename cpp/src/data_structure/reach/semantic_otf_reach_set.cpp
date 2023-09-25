#include <utility>

#include "reachset/utility/reach_operation.hpp"
#include "reach_semantic/data_structure/reach/semantic_otf_reach_set.hpp"
#include "reach_semantic/data_structure/reach/predicates/predicate.hpp"

using namespace semantic_reach;

SemanticOTFReachableSet::SemanticOTFReachableSet(semantic_reach::SemanticConfigurationPtr config,
                                                 collision::CollisionCheckerPtr collision_checker,
                                                 semantic_reach::SemanticModelPtr semantic_model,
                                                 semantic_reach::TrafficRuleInterfacePtr traffic_rule_interface)
        : SemanticReachableSet(std::move(config), std::move(collision_checker), std::move(semantic_model),
                               std::move(traffic_rule_interface)) {
    // Construct finite automaton from traffic rules
    automaton = std::make_unique<FiniteAutomaton>(rule_interface->vec_specifications_ltl,
                                                  this->config->config_traffic_rule.mode_automata);

    // Construct splitter for splitting reachable sets along transitions of the automaton
    splitter = std::make_unique<MintermReachNodeSplitter>(this->semantic_model, this->config);

    // Compute initial reachable set
    SemanticOTFReachableSet::_compute_drivable_area_at_step(step_start);
    SemanticOTFReachableSet::_compute_reachable_set_at_step(step_start);
    _vec_steps_computed.emplace_back(step_start);
}

void SemanticOTFReachableSet::_compute_drivable_area_at_step(const int &step) {
    std::vector<reach::ReachNodePtr> propagated_sets;
    if (step != step_start) {
        auto reachable_set_previous = map_step_to_reachable_set[step - 1];
        if (reachable_set_previous.empty()) {
            map_step_to_drivable_area[step] = {};
            step_to_states_to_drivable_area[step] = {};
            step_to_states_to_propagated_set[step] = {};
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
    // this is done individually for each group calculated above,
    // because we must not merge sets semantically different base sets
    // it is necessary to also consider the states of the propagation source for the partitioning,
    // because only if these are equal, the automaton cannot distinguish the base sets
    // if only the target states were considered, the automaton could possibly distinguish them
    // if the source states reach the target state via different propositions
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
    step_to_states_to_drivable_area[step] = map_states_to_drivable_area;
    step_to_states_to_propagated_set[step] = map_states_to_propagated_set;
    map_step_to_propagated_set[step] = propagated_sets_split;
}

void SemanticOTFReachableSet::_compute_reachable_set_at_step(const int &step) {
    auto states_to_propagated_set = step_to_states_to_propagated_set[step];
    auto states_to_drivable_area = step_to_states_to_drivable_area[step];

    if (states_to_drivable_area.empty()) {
        map_step_to_reachable_set[step] = {};
        return;
    }

    auto num_threads = config->reachable_set().num_threads;

    // work with the reachable sets partitioned by propositions here, because otherwise it could happen
    // that we merge two reachable sets with different propositions when they intersect with the same drivable area
    vector<reach::ReachNodePtr> new_reachable_sets{};
    for (auto const &[automaton_states, drivable_area]: states_to_drivable_area) {
        auto propagated_sets = states_to_propagated_set[automaton_states];

        auto reachable_sets = reach::construct_reach_nodes(drivable_area, propagated_sets, num_threads);

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
        auto [_unused, target_states] = automaton_states;
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
    auto transitions = automaton->multi_transitions_from(current_states);

    std::set<Minterm> minterms{};
    std::transform(transitions.begin(), transitions.end(), std::inserter(minterms, minterms.begin()),
                   [](const auto &transition) {
                       return transition.first;
                   });

    // split and label the reachable set according to the transitions of the automaton
    auto minterm_to_constrained_sets = splitter->split_to_minterms(step, reachable_set, minterms);
    std::vector<reach::ReachNodePtr> constrained_reachable_sets{};
    for (const auto &[minterm, constrained_sets]: minterm_to_constrained_sets) {
        for (const auto &constrained_set: constrained_sets) {
            reachable_set_to_label[constrained_set] = transitions.at(minterm);
            constrained_reachable_sets.emplace_back(constrained_set);
        }
    }

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
