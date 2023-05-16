#include <utility>

#include "reachset/utility/reach_operation.hpp"
#include "reach_semantic/data_structure/reach/semantic_splitting_otf_reach_set.hpp"
#include "reach_semantic/utility/reach_operation.hpp"

using namespace semantic_reach;

SemanticSplittingOTFReachableSet::SemanticSplittingOTFReachableSet(semantic_reach::SemanticConfigurationPtr config,
                                                                   collision::CollisionCheckerPtr collision_checker,
                                                                   semantic_reach::SemanticModelPtr semantic_model,
                                                                   semantic_reach::TrafficRuleInterfacePtr traffic_rule_interface)
        : SemanticReachableSet(std::move(config), std::move(collision_checker), std::move(semantic_model),
                               std::move(traffic_rule_interface)) {
    _initialize_zero_state_polygons();

    // Construct finite automaton from traffic rules
    automaton = std::make_unique<FiniteAutomaton>(rule_interface->get_combined_ltl_specs());

    auto initial_reachable_sets = _construct_initial_reachable_sets();

    std::vector<reach::ReachNodePtr> initial_reachable_sets_split{};
    for (const auto &initial_reachable_set: initial_reachable_sets) {
        auto split_reachable_sets = _split_reachable_set(step_start, initial_reachable_set);
        initial_reachable_sets_split.insert(initial_reachable_sets_split.end(), split_reachable_sets.begin(),
                                            split_reachable_sets.end());
    }
    _filter_reachable_sets(initial_reachable_sets_split, step_start);

    map_step_to_reachable_set[step_start] = initial_reachable_sets_split;
    map_step_to_drivable_area[step_start] = reach::project_base_sets_to_position_domain(
            map_step_to_reachable_set[step_start]);

    _vec_steps_computed.emplace_back(step_start);
}

void SemanticSplittingOTFReachableSet::_compute_drivable_area_at_step(const int &step) {
    auto reachable_set_previous = map_step_to_reachable_set[step - 1];
    if (reachable_set_previous.empty()) {
        map_step_to_drivable_area[step] = {};
        map_step_to_propagated_set[step] = {};
        return;
    }

    auto propagated_sets = _propagate_reachable_set(reachable_set_previous);

    std::vector<reach::ReachNodePtr> propagated_sets_split{};
    for (const auto &propagated_set: propagated_sets) {
        auto split_reachable_sets = _split_reachable_set(step, propagated_set);
        propagated_sets_split.insert(propagated_sets_split.end(), split_reachable_sets.begin(),
                                     split_reachable_sets.end());
    }
    _filter_reachable_sets(propagated_sets_split, step);

    // partition propagated sets by their automaton states
    std::map<std::set<unsigned int>, std::vector<reach::ReachNodePtr>> map_states_to_propagated_set{};
    for (auto const &propagated_set: propagated_sets_split) {
        map_states_to_propagated_set[reachable_set_to_label[propagated_set]].emplace_back(propagated_set);
    }

    // merge, collision check, and repartition propagated sets partitioned by their automaton states,
    // because we must not merge sets with different states
    std::map<std::set<unsigned int>, std::vector<reach::ReachPolygonPtr>> map_states_to_drivable_area{};
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

void SemanticSplittingOTFReachableSet::_compute_reachable_set_at_step(const int &step) {
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
            for (const auto &node: reachable_sets) {
                reachable_set_to_label[node] = automaton_states;
            }
            new_reachable_sets.insert(new_reachable_sets.end(),
                                      std::make_move_iterator(reachable_sets.begin()),
                                      std::make_move_iterator(reachable_sets.end()));
        }
    }
    map_step_to_reachable_set[step] = new_reachable_sets;
}

void
SemanticSplittingOTFReachableSet::_filter_reachable_sets(std::vector<reach::ReachNodePtr> &reachable_sets, int step) {
    bool is_final_step = (step == step_end);
    reachable_sets.erase(std::remove_if(reachable_sets.begin(), reachable_sets.end(),
                                        [this, is_final_step](const reach::ReachNodePtr &node) {
                                            return this->reachable_set_to_label[node].empty() ||
                                                   (is_final_step && !_has_accepting_state(node));
                                        }), reachable_sets.end());
}

bool SemanticSplittingOTFReachableSet::_has_accepting_state(const reach::ReachNodePtr &reachable_set) {
    auto states = reachable_set_to_label[reachable_set];
    return std::any_of(states.begin(), states.end(), [this](const int &state) {
        return automaton->is_accepting_state(state);
    });
}

std::vector<reach::ReachNodePtr>
SemanticSplittingOTFReachableSet::_split_reachable_set(int step, const reach::ReachNodePtr &reachable_set) {
    std::vector<reach::ReachNodePtr> split_sets{};
    auto current_states =
            step == step_start ? std::set<unsigned int>{automaton->initial_state()} : reachable_set_to_label[reachable_set];
    std::map<Minterm, std::vector<reach::ReachNodePtr>> minterm_to_constrained_sets{};

    for (const auto &[next_state, minterms]: automaton->combined_transitions_from(current_states)) {
        for (const auto &minterm: minterms) {
            std::vector<reach::ReachNodePtr> constrained_reachable_sets;
            // if we saw that minterm already, reuse the constrained sets
            auto it = minterm_to_constrained_sets.find(minterm);
            if (it != minterm_to_constrained_sets.end()) {
                constrained_reachable_sets = it->second;
            } else {
                constrained_reachable_sets = _split_reachable_set_to_minterm(step, reachable_set, minterm);
                minterm_to_constrained_sets[minterm] = constrained_reachable_sets;
            }

            for (const auto &constrained_reachable_set: constrained_reachable_sets) {
                reachable_set_to_label[constrained_reachable_set].insert(next_state);
            }
            split_sets.insert(split_sets.end(), constrained_reachable_sets.begin(), constrained_reachable_sets.end());
        }
    }

    return split_sets;
}

std::vector<reach::ReachNodePtr>
SemanticSplittingOTFReachableSet::_split_reachable_set_to_minterm(int step, reach::ReachNodePtr reachable_set,
                                                                  semantic_reach::Minterm minterm) {
    // TODO
    return {reachable_set};
}
