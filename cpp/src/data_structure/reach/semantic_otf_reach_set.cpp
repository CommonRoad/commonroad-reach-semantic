#include <utility>

#include "reachset/utility/reach_operation.hpp"
#include "reach_semantic/data_structure/reach/semantic_otf_reach_set.hpp"
#include "reach_semantic/utility/reach_operation.hpp"

using namespace semantic_reach;

SemanticOTFReachableSet::SemanticOTFReachableSet(semantic_reach::SemanticConfigurationPtr config,
                                                 collision::CollisionCheckerPtr collision_checker,
                                                 semantic_reach::SemanticModelPtr semantic_model,
                                                 semantic_reach::TrafficRuleInterfacePtr traffic_rule_interface)
        : SemanticReachableSet(std::move(config), std::move(collision_checker), std::move(semantic_model),
                               std::move(traffic_rule_interface)) {
    _initialize_zero_state_polygons();

    // Construct finite automaton from traffic rules
    automaton = std::make_unique<FiniteAutomaton>(rule_interface->vec_specifications_ltl, this->config->config_traffic_rule.mode_automata);

    auto initial_reachable_sets = _construct_initial_reachable_sets();

    // Label initial state with propositions and automaton states
    labeler->label_initial_state(initial_reachable_sets, step_start);
    _label_reachable_sets_with_automaton_states(initial_reachable_sets, true);
    _filter_reachable_sets(initial_reachable_sets, step_start);

    map_step_to_reachable_set[step_start] = initial_reachable_sets;
    map_step_to_drivable_area[step_start] = reach::project_base_sets_to_position_domain(
            map_step_to_reachable_set[step_start]);

    _vec_steps_computed.emplace_back(step_start);
}

void SemanticOTFReachableSet::_compute_drivable_area_at_step(const int &step) {
    auto reachable_set_previous = map_step_to_reachable_set[step - 1];
    if (reachable_set_previous.empty()) {
        map_step_to_drivable_area[step] = {};
        map_step_to_propagated_set[step] = {};
        return;
    }

    auto propagated_sets = _propagate_reachable_set(reachable_set_previous);

    // split w.r.t regions and position intervals
    propagated_sets = labeler->split_wrt_regions(step, propagated_sets);
    propagated_sets = labeler->split_wrt_position_intervals(step, propagated_sets);

    // discard the ones colliding with vehicles
    propagated_sets = labeler->discard_colliding_nodes(propagated_sets);

    // examine whether the propagated sets satisfy TPL specifications
    propagated_sets = rule_interface->examine_tpl_specifications(step, propagated_sets,
                                                                 labeler->reachable_set_to_propositions);

    // update traffic propositions of the propagated sets
    propagated_sets = labeler->label_traffic_propositions(step, propagated_sets);

    _label_reachable_sets_with_automaton_states(propagated_sets);
    _filter_reachable_sets(propagated_sets, step);

    // partition propagated sets by their automaton states
    std::map<std::pair<std::set<unsigned int>, std::set<unsigned int>>, std::vector<reach::ReachNodePtr>> map_states_to_propagated_set{};
    for (auto const &propagated_set: propagated_sets) {
        auto key = std::make_pair(reachable_set_to_label[propagated_set->vec_nodes_source[0]],
                                  reachable_set_to_label[propagated_set]);
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
    map_step_to_propagated_set[step] = propagated_sets;
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
    bool discard_small_node = (num_drivable_area > 1);

    // work with the reachable sets partitioned by propositions here, because otherwise it could happen
    // that we merge two reachable sets with different propositions when they intersect with the same drivable area
    vector<reach::ReachNodePtr> new_reachable_sets{};
    for (auto const &[automaton_states, drivable_area]: map_states_to_drivable_area) {
        auto propagated_set = map_states_to_propagated_set[automaton_states];

        auto vec_nodes = reach::construct_reach_nodes(drivable_area, propagated_set, num_threads);

        if (discard_small_node) {
            vec_nodes = semantic_reach::discard_nodes_with_short_edge(vec_nodes,
                                                                      config->reachable_set().length_edge_node_min);
        }

        if (!vec_nodes.empty()) {
            auto reachable_sets = reach::connect_children_to_parents(step, vec_nodes, num_threads);
            // assign label to all newly constructed reach nodes
            auto [_, target_states] = automaton_states;
            for (const auto &node: reachable_sets) {
                reachable_set_to_label[node] = target_states;
            }
            new_reachable_sets.insert(new_reachable_sets.end(),
                                      std::make_move_iterator(reachable_sets.begin()),
                                      std::make_move_iterator(reachable_sets.end()));
        }
    }
    map_step_to_reachable_set[step] = new_reachable_sets;
}

void
SemanticOTFReachableSet::_label_reachable_sets_with_automaton_states(std::vector<reach::ReachNodePtr> &reachable_sets,
                                                                     bool initial_step) {
    for (const auto &reachable_set: reachable_sets) {
        auto automaton_states = initial_step ? std::set<unsigned int>{automaton->initial_state()}
                                             : reachable_set_to_label[reachable_set->vec_nodes_source[0]];
        for (const auto &automaton_state: automaton_states) {
            _label_automaton_states(reachable_set, automaton_state);
        }
    }
}

void
SemanticOTFReachableSet::_label_automaton_states(const reach::ReachNodePtr &reachable_set, unsigned int current_state) {
    auto reach_props = labeler->reachable_set_to_propositions[reachable_set].set_propositions;
    for (const auto &[minterms, next_state]: automaton->transitions_from(current_state)) {
        for (const auto &minterm: minterms) {
            std::vector<std::string> positive_props{};
            std::vector<std::string> negative_props{};
            for (const auto &[proposition, negated]: minterm) {
                if (negated) {
                    negative_props.emplace_back(proposition);
                } else {
                    positive_props.emplace_back(proposition);
                }
            }
            // check if positive props are subset of reach props and negative props are disjoint
            if (std::includes(reach_props.begin(), reach_props.end(), positive_props.begin(), positive_props.end()) &&
                std::all_of(negative_props.begin(), negative_props.end(),
                            [&reach_props](const std::string &prop) {
                                // return prop \notin reach_props;
                                return reach_props.find(prop) == reach_props.end();
                            })) {
                reachable_set_to_label[reachable_set].insert(next_state);
                break; // inner loop
            }
        }
    }
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
