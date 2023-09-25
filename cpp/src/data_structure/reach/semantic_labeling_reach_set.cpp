#include "reach_semantic/data_structure/reach/semantic_labeling_reach_set.hpp"

#include <utility>
#include "reachset/utility/shared_using.hpp"
#include "reachset/utility/reach_operation.hpp"

using namespace semantic_reach;

SemanticLabelingReachableSet::SemanticLabelingReachableSet(SemanticConfigurationPtr config,
                                                           CollisionCheckerPtr collision_checker,
                                                           SemanticModelPtr semantic_model,
                                                           TrafficRuleInterfacePtr traffic_rule_interface) :
        SemanticReachableSet(std::move(config), std::move(collision_checker), std::move(semantic_model),
                             std::move(traffic_rule_interface)) {
    map_step_to_reachable_set[step_start] = _construct_initial_reachable_sets();
    map_step_to_drivable_area[step_start] = reach::project_base_sets_to_position_domain(
            map_step_to_reachable_set[step_start]);
    labeler->label_initial_state(map_step_to_reachable_set[step_start], step_start);
    _initialize_zero_state_polygons();

    _vec_steps_computed.emplace_back(step_start);
}

/// *Steps*:
/// 1. Propagate each node of the reachable set from the previous step. This forms a list of propagated sets.
/// 2. Split the propagated ses with respect to regions and position intervals. Discard colliding sets.
/// 3. Label nodes with relevant atomic propositions.
/// 4. Examine Time-labeled Propositional Logic specifications.
/// 5. Compute collision-free drivable areas.
/// 3. Merge and repartition these rectangles to reduce computation load.
/// 4. Check for collision and split the repartitioned rectangles into collision-free rectangles.
/// 5. Merge and repartition the collision-free rectangles again to reduce number of nodes.
void SemanticLabelingReachableSet::_compute_drivable_area_at_step(int const &step) {
    auto reachable_set_previous = map_step_to_reachable_set[step - 1];
    if (reachable_set_previous.empty()) {
        map_step_to_propositions_to_propagated_set[step] = {};
        map_step_to_propositions_to_drivable_area[step] = {};
        return;
    }

    auto vec_propagated_set = _propagate_reachable_set(reachable_set_previous);

    // split w.r.t regions and position intervals
    vec_propagated_set = labeler->split_wrt_regions(step, vec_propagated_set);
    vec_propagated_set = labeler->split_wrt_position_intervals(step, vec_propagated_set);

    // discard the ones colliding with vehicles
    vec_propagated_set = labeler->discard_colliding_nodes(vec_propagated_set);

    // examine whether the propagated sets satisfy TPL specifications
    vec_propagated_set = rule_interface->examine_tpl_specifications(step, vec_propagated_set,
                                                                    labeler->reachable_set_to_propositions);

    // update traffic propositions of the propagated sets
    vec_propagated_set = labeler->label_traffic_propositions(step, vec_propagated_set);

    // partition propagated sets by their propositions
    unordered_map<PropositionHolder, vector<reach::ReachNodePtr>, PropositionHolder::HashFunction>
            dict_propositions_to_propagated_set{};
    for (auto const &propagated_set: vec_propagated_set) {
        dict_propositions_to_propagated_set[labeler->reachable_set_to_propositions[propagated_set]].emplace_back(
                propagated_set);
    }

    // merge, collision check, and repartition propagated sets partitioned by their propositions,
    // because we must not merge sets with different propositions
    unordered_map<PropositionHolder, vector<reach::ReachPolygonPtr>, PropositionHolder::HashFunction>
            dict_propositions_to_drivable_area{};
    std::vector<reach::ReachPolygonPtr> vec_drivable_area{};
    for (const auto &[propositions, propagated_sets_per_proposition]: dict_propositions_to_propagated_set) {
        auto vec_rectangles_projected = reach::project_base_sets_to_position_domain(propagated_sets_per_proposition);
        auto drivable_area_at_proposition = _collision_check_and_repartition(vec_rectangles_projected, step);
        dict_propositions_to_drivable_area[propositions] = drivable_area_at_proposition;
        vec_drivable_area.insert(vec_drivable_area.end(), drivable_area_at_proposition.begin(),
                                 drivable_area_at_proposition.end());
    }

    map_step_to_drivable_area[step] = vec_drivable_area;
    map_step_to_propositions_to_drivable_area[step] = dict_propositions_to_drivable_area;
    map_step_to_propositions_to_propagated_set[step] = dict_propositions_to_propagated_set;
    map_step_to_propagated_set[step] = vec_propagated_set;
}

/// *Steps*:
/// 1. construct reach nodes from drivable area and the propagated base sets.
/// 2. update parent-child relationship of the nodes.
void SemanticLabelingReachableSet::_compute_reachable_set_at_step(int const &step) {
    auto map_propositions_to_propagated_set = map_step_to_propositions_to_propagated_set[step];
    auto map_propositions_to_drivable_area = map_step_to_propositions_to_drivable_area[step];

    if (map_propositions_to_drivable_area.empty()) {
        map_step_to_reachable_set[step] = {};
        return;
    }

    auto num_threads = config->reachable_set().num_threads;

    // work with the reachable sets partitioned by propositions here, because otherwise it could happen
    // that we merge two reachable sets with different propositions when they intersect with the same drivable area

    vector<reach::ReachNodePtr> new_reachable_sets{};
    for (auto const &[proposition_holder, drivable_area]: map_propositions_to_drivable_area) {
        auto propagated_set = map_propositions_to_propagated_set[proposition_holder];

        auto vec_nodes = reach::construct_reach_nodes(drivable_area, propagated_set, num_threads);

        if (!vec_nodes.empty()) {
            auto reachable_sets = reach::connect_children_to_parents(step, vec_nodes, num_threads);
            // copy propositions for newly constructed nodes. Because all propagated sets are labeled with the same
            // propositions, we simply use the first as reference.
            labeler->copy_labels(propagated_set[0], reachable_sets);
            new_reachable_sets.insert(new_reachable_sets.end(),
                                      std::make_move_iterator(reachable_sets.begin()),
                                      std::make_move_iterator(reachable_sets.end()));
        }
    }
    map_step_to_reachable_set[step] = new_reachable_sets;
}
