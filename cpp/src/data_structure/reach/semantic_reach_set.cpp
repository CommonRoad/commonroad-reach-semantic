#include "reach_semantic/data_structure/reach/semantic_reach_set.hpp"

#include <utility>
#include "reachset/utility/shared_using.hpp"
#include "reachset/utility/reach_operation.hpp"
#include "reach_semantic/utility/reach_operation.hpp"

using namespace semantic_reach;

SemanticReachableSet::SemanticReachableSet(SemanticConfigurationPtr config) : config(std::move(config)) {
    _initialize();
}

SemanticReachableSet::SemanticReachableSet(SemanticConfigurationPtr config, CollisionCheckerPtr collision_checker,
                                           SemanticModelPtr semantic_model) :
        config(std::move(config)), collision_checker(std::move(collision_checker)),
        semantic_model(std::move(semantic_model)) {
    _initialize();
}

SemanticReachableSet::SemanticReachableSet(SemanticConfigurationPtr config, CollisionCheckerPtr collision_checker,
                                           SemanticModelPtr semantic_model, TrafficRuleInterfacePtr traffic_rule_interface) :
        config(std::move(config)), collision_checker(std::move(collision_checker)),
        semantic_model(std::move(semantic_model)), rule_interface(std::move(traffic_rule_interface)) {
    _initialize();
}

void SemanticReachableSet::_initialize() {
    labeler = std::make_shared<ReachableSetLabeler>(semantic_model);

    step_start = config->planning().step_start;
    step_end = step_start + config->planning().steps_computation;

    map_step_to_reachable_set[step_start] = _construct_initial_reachable_sets();
    map_step_to_drivable_area[step_start] = semantic_reach::project_propagated_sets_to_position_domain(map_step_to_reachable_set[step_start]);
    _initialize_zero_state_polygons();

    _vec_steps_computed.emplace_back(step_start);
}

/// @note Computation of the reachable set of an LTI system requires the zero-state response of the system.
void SemanticReachableSet::_initialize_zero_state_polygons() {
    polygon_zero_state_lon = create_zero_state_polygon(config->planning().dt,
                                                       config->vehicle().ego.a_lon_min,
                                                       config->vehicle().ego.a_lon_max);

    polygon_zero_state_lat = create_zero_state_polygon(config->planning().dt,
                                                       config->vehicle().ego.a_lat_min,
                                                       config->vehicle().ego.a_lat_max);
}

std::vector<SemanticReachNodePtr> SemanticReachableSet::_construct_initial_reachable_sets() {
    // initial drivable area
    auto tuple_vertices = generate_tuple_vertices_position_rectangle_initial(config);

    // initial reachable set
    auto [tuple_vertices_polygon_lon, tuple_vertices_polygon_lat] =
            generate_tuples_vertices_polygons_initial(config);
    auto polygon_lon = make_shared<reach::ReachPolygon>(tuple_vertices_polygon_lon);
    auto polygon_lat = make_shared<reach::ReachPolygon>(tuple_vertices_polygon_lat);

    return {std::make_shared<SemanticReachNode>(step_start, polygon_lon, polygon_lat, PropositionHolder())};
}

void SemanticReachableSet::compute(int step_start, int step_end) {
    if (step_start == 0) step_start = this->step_start + 1;
    if (step_end == 0) step_end = this->step_end;

    for (auto step = step_start; step < step_end + 1; step++) {
        _compute_drivable_area_at_step(step);
        _compute_reachable_set_at_step(step);
        _vec_steps_computed.emplace_back(step);
    }

    //if (step_start != step_end and config->reachable_set().prune_nodes) {
    //    prune_nodes_not_reaching_final_step();
    //}
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
void SemanticReachableSet::_compute_drivable_area_at_step(int const& step) {
    auto reachable_set_previous = map_step_to_reachable_set[step - 1];
    if (reachable_set_previous.empty()) {
        map_step_to_propositions_to_propagated_set[step] = {};
        map_step_to_propositions_to_drivable_area[step] = {};
        return;
    }

    auto vec_propagated_set = _propagate_reachable_set(reachable_set_previous);

    // split w.r.t regions and position intervals
    auto vec_propagated_set_split = labeler->split_wrt_regions(step, vec_propagated_set);
    vec_propagated_set_split = labeler->split_wrt_position_intervals(step, vec_propagated_set_split);

    // discard the ones colliding with vehicles
    vec_propagated_set_split = labeler->discard_colliding_nodes(vec_propagated_set_split);

    // examine whether the propagated sets satisfy TPL specifications
    vec_propagated_set = rule_interface->examine_tpl_specifications(step, vec_propagated_set);

    // update traffic propositions of the propagated sets
    vec_propagated_set = labeler->label_traffic_propositions(step, vec_propagated_set_split);

    // partition propagated sets by their propositions
    unordered_map<PropositionHolder, vector<SemanticReachNodePtr>, PropositionHolder::HashFunction>
            dict_propositions_to_propagated_set{};
    for (auto const& propagated_set: vec_propagated_set) {
        dict_propositions_to_propagated_set[labeler->reachable_set_to_propositions[propagated_set]].emplace_back(propagated_set);
    }

    // merge, collision check, and repartition propagated sets partitioned by their propositions,
    // because we must not merge sets with different propositions
    unordered_map<PropositionHolder, vector<reach::ReachPolygonPtr>, PropositionHolder::HashFunction>
            dict_propositions_to_drivable_area{};
    std::vector<reach::ReachPolygonPtr> vec_drivable_area{};
    for (const auto &[propositions, propagated_sets_per_proposition]: dict_propositions_to_propagated_set) {
        auto vec_rectangles_projected = project_propagated_sets_to_position_domain(propagated_sets_per_proposition);
        auto drivable_area_at_proposition = _collision_check_and_repartition(vec_rectangles_projected, step);
        dict_propositions_to_drivable_area[propositions] = drivable_area_at_proposition;
        vec_drivable_area.insert(vec_drivable_area.end(), drivable_area_at_proposition.begin(),drivable_area_at_proposition.end());
    }

    map_step_to_drivable_area[step] = vec_drivable_area;
    map_step_to_propositions_to_drivable_area[step] = dict_propositions_to_drivable_area;
    map_step_to_propositions_to_propagated_set[step] = dict_propositions_to_propagated_set;
    map_step_to_propagated_set[step] = vec_propagated_set;
}


vector<SemanticReachNodePtr> SemanticReachableSet::_propagate_reachable_set(vector<SemanticReachNodePtr> const& vec_nodes) {
    vector<SemanticReachNodePtr> vec_base_sets_propagated;
    vec_base_sets_propagated.reserve(vec_nodes.size());

#pragma omp parallel num_threads(config->reachable_set().num_threads) \
default(none) shared(vec_nodes, vec_base_sets_propagated)
    {
        vector<SemanticReachNodePtr> vec_base_sets_propagated_thread;
        vec_base_sets_propagated_thread.reserve(vec_nodes.size());

#pragma omp for nowait
        for (auto const& node: vec_nodes) {
            try {
                auto polygon_lon_propagated = propagate_polygon(node->polygon_lon,
                                                                polygon_zero_state_lon,
                                                                config->planning().dt,
                                                                config->vehicle().ego.v_lon_min,
                                                                config->vehicle().ego.v_lon_max);

                auto polygon_lat_propagated = propagate_polygon(node->polygon_lat,
                                                                polygon_zero_state_lat,
                                                                config->planning().dt,
                                                                config->vehicle().ego.v_lat_min,
                                                                config->vehicle().ego.v_lat_max);

                auto propagated_set = make_shared<SemanticReachNode>(node->step,
                                                             polygon_lon_propagated,
                                                             polygon_lat_propagated,
                                                             PropositionHolder());
                propagated_set->vec_nodes_source.emplace_back(node);
                vec_base_sets_propagated_thread.emplace_back(propagated_set);
            }
            catch (std::exception& e) {
                continue;
            }
        }
#pragma omp critical
        vec_base_sets_propagated.insert(vec_base_sets_propagated.end(),
                                        std::make_move_iterator(vec_base_sets_propagated_thread.begin()),
                                        std::make_move_iterator(vec_base_sets_propagated_thread.end()));
    }
    return vec_base_sets_propagated;
}

std::vector<reach::ReachPolygonPtr>
SemanticReachableSet::_collision_check_and_repartition(std::vector<reach::ReachPolygonPtr> rectangles, int const &step) {
    auto mode_repartition = config->reachable_set().mode_repartition;
    auto size_grid = config->reachable_set().size_grid;
    auto size_grid_2nd = config->reachable_set().size_grid_2nd;
    auto radius_terminal_split = config->reachable_set().radius_terminal_split;

    vector<reach::ReachPolygonPtr> drivable_area{};
    // repartition, then collision check
    if (mode_repartition == 1) {
        // create repartitioned rectangles from the projected base sets
        auto vec_rectangles_repartitioned = create_repartitioned_rectangles(rectangles, size_grid);
        drivable_area = check_collision_and_split_rectangles(step, collision_checker,
                                                             vec_rectangles_repartitioned,
                                                             radius_terminal_split,
                                                             config->reachable_set().num_threads);
    }

    // collision check, then repartition
    else if (mode_repartition == 2) {
        auto vec_rectangles_collision_free = \
                    check_collision_and_split_rectangles(step, collision_checker,
                                                         rectangles,
                                                         radius_terminal_split,
                                                         config->reachable_set().num_threads);
        drivable_area = create_repartitioned_rectangles(vec_rectangles_collision_free, size_grid);
    }

    // repartition, collision check, then repartition again
    else if (mode_repartition == 3) {
        auto vec_rectangles_repartitioned = create_repartitioned_rectangles(rectangles, size_grid);

        auto vec_rectangles_collision_free = \
                    check_collision_and_split_rectangles(step, collision_checker,
                                                         vec_rectangles_repartitioned,
                                                         radius_terminal_split,
                                                         config->reachable_set().num_threads);

        drivable_area = create_repartitioned_rectangles(vec_rectangles_collision_free,
                                                        size_grid_2nd);
    } else {
        throw (std::logic_error("Invalid mode for repartition."));
    }

    return drivable_area;
}

/// *Steps*:
/// 1. construct reach nodes from drivable area and the propagated base sets.
/// 2. update parent-child relationship of the nodes.
void SemanticReachableSet::_compute_reachable_set_at_step(int const& step) {
    auto map_propositions_to_propagated_set = map_step_to_propositions_to_propagated_set[step];
    auto map_propositions_to_drivable_area = map_step_to_propositions_to_drivable_area[step];

    if (map_propositions_to_drivable_area.empty()) {
        map_step_to_reachable_set[step] = {};
        return;
    }

    auto num_threads = config->reachable_set().num_threads;

    // discard drivable area with small area if there are more than one node (this is subject to change)
    unsigned long num_drivable_area = 0;
    for (auto const& [proposition_holder, drivable_area]: map_propositions_to_drivable_area) {
        num_drivable_area += drivable_area.size();
    }
    bool discard_small_node = (num_drivable_area > 1);

    // work with the reachable sets partitioned by propositions here, because otherwise it could happen
    // that we merge two reachable sets with different propositions when they intersect with the same drivable area

    vector<SemanticReachNodePtr> new_reachable_sets{};
    for (auto const& [proposition_holder, drivable_area]: map_propositions_to_drivable_area) {
        auto propagated_set = map_propositions_to_propagated_set[proposition_holder];

        auto vec_nodes = semantic_reach::construct_reach_nodes(drivable_area, propagated_set, num_threads);

        if (discard_small_node) {
            vec_nodes = semantic_reach::discard_nodes_with_short_edge(vec_nodes, config->reachable_set().length_edge_node_min);
        }

        if (!vec_nodes.empty()) {
            auto reachable_sets = semantic_reach::connect_children_to_parents(step, vec_nodes, num_threads);
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
//
///// Iterates through reachability graph backward in time, discards nodes that don't have a child node.
//void SemanticReachableSet::prune_nodes_not_reaching_final_step() {
//    auto cnt_nodes_before_pruning = reachable_set_at_step(step_end).size();
//    auto cnt_nodes_after_pruning = cnt_nodes_before_pruning;
//
//    for (auto step = step_end - 1; step > step_start - 1; step--) {
//        auto vec_nodes = reachable_set_at_step(step);
//        cnt_nodes_before_pruning += vec_nodes.size();
//
//        vector<int> vec_idx_nodes_to_be_deleted{};
//        for (int idx_node = 0; idx_node < vec_nodes.size(); idx_node++) {
//            // discard the node if it has no child node
//            auto node = vec_nodes[idx_node];
//            if (node->vec_nodes_child().empty()) {
//                vec_idx_nodes_to_be_deleted.push_back(idx_node);
//                // iterate through its parent nodes and disconnect them
//                for (auto const& node_parent: node->vec_nodes_parent()) {
//                    node_parent->remove_child_node(node);
//                }
//            }
//        }
//        // discard nodes without a child
//        vector<reach::ReachPolygonPtr> vec_drivable_area_updated{};
//        vector<SemanticReachNodePtr> vec_reachable_set_updated{};
//        for (int idx_node = 0; idx_node < vec_nodes.size(); idx_node++) {
//            auto result = std::find(vec_idx_nodes_to_be_deleted.begin(),
//                                    vec_idx_nodes_to_be_deleted.end(),
//                                    idx_node) != vec_idx_nodes_to_be_deleted.end();
//
//            if (not result) {
//                auto node = vec_nodes[idx_node];
//                vec_drivable_area_updated.emplace_back(node->position_rectangle());
//                vec_reachable_set_updated.emplace_back(node);
//            }
//        }
//        // update drivable area and reachable set dictionaries
//        map_step_to_drivable_area[step] = vec_drivable_area_updated;
//        map_step_to_reachable_set[step] = vec_reachable_set_updated;
//        cnt_nodes_after_pruning += map_step_to_reachable_set[step].size();
//    }
//
//    _pruned = true;
//    // cout << "\t#Nodes before pruning: \t" << cnt_nodes_before_pruning << endl;
//    // cout << "\t#Nodes after pruning: \t" << cnt_nodes_after_pruning << endl;
//}

vector<SemanticReachNodePtr> SemanticReachableSet::_call_python_dummy(int const& step, vector<SemanticReachNodePtr> const& vec_nodes) {
    vector<SemanticReachNodePtr> vec_nodes_new{};
    for (auto const& node: vec_nodes) {
        vec_nodes_new.emplace_back(semantic_model->obj_semantic_model_py.attr("call_python_dummy")(step, node)
                                           .cast<SemanticReachNodePtr>());
    }
    return vec_nodes_new;
}