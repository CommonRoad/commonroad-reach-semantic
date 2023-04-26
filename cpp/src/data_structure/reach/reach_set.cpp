#include "reach_semantic/data_structure/reach/reach_set.hpp"

#include <utility>
#include "reach_semantic/utility/shared_using.hpp"
#include "reach_semantic/utility/reach_operation.hpp"

using namespace reach;

ReachableSet::ReachableSet(ConfigurationPtr config) : config(std::move(config)) {
    _initialize();
}

ReachableSet::ReachableSet(ConfigurationPtr config, CollisionCheckerPtr collision_checker,
                           SemanticModelPtr semantic_model) :
        config(std::move(config)), collision_checker(std::move(collision_checker)),
        semantic_model(std::move(semantic_model)) {
    _initialize();
}

ReachableSet::ReachableSet(ConfigurationPtr config, CollisionCheckerPtr collision_checker,
                           SemanticModelPtr semantic_model, TrafficRuleInterfacePtr traffic_rule_interface) :
        config(std::move(config)), collision_checker(std::move(collision_checker)),
        semantic_model(std::move(semantic_model)), rule_interface(std::move(traffic_rule_interface)) {
    _initialize();
}

void ReachableSet::_initialize() {
    step_start = config->planning().step_start;
    step_end = step_start + config->planning().steps_computation;

    _initialize_zero_state_polygons();
    _construct_initial_drivable_area_and_reachable_set();

    _vec_steps_computed.emplace_back(step_start);
}

/// @note Computation of the reachable set of an LTI system requires the zero-state response of the system.
void ReachableSet::_initialize_zero_state_polygons() {
    polygon_zero_state_lon = create_zero_state_polygon(config->planning().dt,
                                                       config->vehicle().ego.a_lon_min,
                                                       config->vehicle().ego.a_lon_max);

    polygon_zero_state_lat = create_zero_state_polygon(config->planning().dt,
                                                       config->vehicle().ego.a_lat_min,
                                                       config->vehicle().ego.a_lat_max);
}

void ReachableSet::_construct_initial_drivable_area_and_reachable_set() {
    // initial drivable area
    auto tuple_vertices = generate_tuple_vertices_position_rectangle_initial(config);
    auto drivable_area_initial = make_shared<ReachPolygon>(std::get<0>(tuple_vertices),
                                                           std::get<1>(tuple_vertices),
                                                           std::get<2>(tuple_vertices),
                                                           std::get<3>(tuple_vertices));
    // initial reachable set
    auto [tuple_vertices_polygon_lon, tuple_vertices_polygon_lat] =
            generate_tuples_vertices_polygons_initial(config);
    auto polygon_lon = make_shared<ReachPolygon>(tuple_vertices_polygon_lon);
    auto polygon_lat = make_shared<ReachPolygon>(tuple_vertices_polygon_lat);

    // obtain initial propositions
    auto proposition_holder = obtain_propositions_for_rectangle(drivable_area_initial, 0);
    auto node_initial = make_shared<ReachNode>(config->planning().step_start,
                                               polygon_lon,
                                               polygon_lat,
                                               proposition_holder);

    node_initial = label_traffic_propositions(0, vector<ReachNodePtr>{node_initial})[0];

    map_step_to_propositions_to_drivable_area[0][proposition_holder].emplace_back(drivable_area_initial);
    map_step_to_propositions_to_reachable_set[0][proposition_holder].emplace_back(node_initial);
}

/// Intersects the rectangle with regions and position intervals.
PropositionHolder
ReachableSet::obtain_propositions_for_rectangle(ReachPolygonPtr const& rectangle, int const& step) const {
    auto proposition_holder = PropositionHolder();

    /// retrieve propositions from the intersecting lanelet region
    for (auto const& region: semantic_model->vec_regions) {
        if (region->intersects(rectangle) and region->polygon_cvln->intersects(rectangle)) {
            for (auto const& [group, set_propositions]:
                    region->map_group_to_propositions_at_step(step)) {
                proposition_holder.add_propositions(set_propositions, group);
            }
            break;
        }
    }

    /// retrieve vehicle-related propositions from position intervals
    auto vec_intervals_lon = semantic_model->map_step_to_position_intervals[0]["lon"];
    auto vec_intervals_lat = semantic_model->map_step_to_position_intervals[0]["lat"];

    for (auto const& interval_lon: vec_intervals_lon) {
        if (interval_lon->intersects(rectangle->p_lon_min(), rectangle->p_lon_max())) {
            proposition_holder.add_propositions(interval_lon->set_propositions, PropositionGroup::POSITION);
            break;
        }
    }

    for (auto const& interval_lat: vec_intervals_lat) {
        if (interval_lat->intersects(rectangle->p_lat_min(), rectangle->p_lat_max())) {
            proposition_holder.add_propositions(interval_lat->set_propositions, PropositionGroup::POSITION);
            break;
        }
    }

    return proposition_holder;
}

vector<ReachNodePtr> ReachableSet::label_traffic_propositions(int const& step, vector<ReachNodePtr> vec_nodes) {
    try {
        auto vec_nodes_labeled =
                semantic_model->obj_semantic_model_py.attr("label_traffic_propositions")(step, vec_nodes)
                        .cast<vector<ReachNodePtr>>();

        return vec_nodes_labeled;
    }
    catch (py::error_already_set& e) {
        cout << "Function: label_traffic_propositions" << endl;
        py::print(e.what());

        return {};
    }
}

//vector<ReachNodePtr> ReachableSet::examine_tpl_specifications(int const& step, vector<ReachNodePtr> vec_nodes) {
//    try {
//        auto vec_nodes_labeled =
//                semantic_model->obj_semantic_model_py.attr("label_traffic_propositions")(step, vec_nodes)
//                        .cast<vector<ReachNodePtr>>();
//
//        return vec_nodes_labeled;
//    }
//    catch (py::error_already_set& e) {
//        cout << "Function: label_traffic_propositions" << endl;
//        py::print(e.what());
//
//        return {};
//    }
//}

void ReachableSet::compute(int step_start, int step_end) {
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
void ReachableSet::_compute_drivable_area_at_step(int const& step) {
    auto map_propositions_to_reachable_set_previous = map_step_to_propositions_to_reachable_set[step - 1];
    if (map_propositions_to_reachable_set_previous.empty()) {
        map_step_to_propositions_to_propagated_set[step] = {};
        map_step_to_propositions_to_drivable_area[step] = {};
        return;
    }

    unordered_map<PropositionHolder, vector<ReachNodePtr>, PropositionHolder::HashFunction>
            dict_proposition_holder_to_propagated_set{};

    // iterate through list of nodes with different sets of propositions
    for (auto const& [propositions, vec_nodes]: map_propositions_to_reachable_set_previous) {
        auto vec_propagated_set = _propagate_reachable_set(vec_nodes);
        // split w.r.t regions and position intervals
        auto vec_propagated_set_split = _split_wrt_regions(step, vec_propagated_set);
        vec_propagated_set_split = _split_wrt_intervals(step, vec_propagated_set_split);

        // discard the ones colliding with vehicles
        vec_propagated_set_split = _discard_colliding_nodes(vec_propagated_set_split);

        // update traffic propositions of the propagated sets
        vec_propagated_set = label_traffic_propositions(step, vec_propagated_set_split);

        // examine whether the propagated sets satisfy TPL specifications
        vec_propagated_set = rule_interface->examine_tpl_specifications(step, vec_propagated_set);

        for (auto const& propagated_set: vec_propagated_set) {
            dict_proposition_holder_to_propagated_set[propagated_set->proposition_holder].emplace_back(propagated_set);
        }
    }
    // compute collision-free drivable areas
    auto dict_propositions_to_drivable_area =
            _compute_collision_free_drivable_area(step, dict_proposition_holder_to_propagated_set);

    map_step_to_propositions_to_propagated_set[step] = dict_proposition_holder_to_propagated_set;
    map_step_to_propositions_to_drivable_area[step] = dict_propositions_to_drivable_area;
}


vector<ReachNodePtr> ReachableSet::_propagate_reachable_set(vector<ReachNodePtr> const& vec_nodes) {
    vector<ReachNodePtr> vec_base_sets_propagated;
    vec_base_sets_propagated.reserve(vec_nodes.size());

#pragma omp parallel num_threads(config->reachable_set().num_threads) \
default(none) shared(vec_nodes, vec_base_sets_propagated)
    {
        vector<ReachNodePtr> vec_base_sets_propagated_thread;
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

                auto propagated_set = make_shared<ReachNode>(node->step,
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

/// *Steps*:
/// 1. intersect propagated sets in the position domain with lanelet regions
/// 2. over-approximate and restore to axis-aligned rectangles
vector<ReachNodePtr> ReachableSet::_split_wrt_regions(int const& step, vector<ReachNodePtr> const& vec_nodes) {
    if (vec_nodes.empty()) {
        return {};
    }

    vector<ReachNodePtr> vec_nodes_split = {};
    // iterate through region and examine propagated sets that are intersecting with the region
    for (auto const& region: semantic_model->vec_regions) {
        for (auto const& node: vec_nodes) {

            auto rectangle = node->position_rectangle();
            // there is no possibility of intersection
            if (!region->intersects(rectangle, "CVLN")) continue;

            vector<ReachPolygonPtr> vec_polygons_intersection{};
            // there is a possibility of intersection
            try {
                vec_polygons_intersection = region->polygon_cvln->intersection(rectangle);
            }
            catch (...) {
                continue;
            }
            if (vec_polygons_intersection.empty()) continue;

            // over-approximate by restoring to axis-aligned rectangles
            auto [p_lon_min, p_lat_min, p_lon_max, p_lat_max] =
                    obtain_extremum_coordinates_of_polygons(vec_polygons_intersection);
            try {
                // clone the propagated set and split in the position domain, update the propositions
                auto node_new = node->clone();
                node_new->intersect_in_position_domain(p_lon_min, p_lat_min, p_lon_max, p_lat_max);
                vec_nodes_split.emplace_back(update_propositions_with_region(node_new, region, step));
            }
            catch (py::error_already_set& e) {
                cout << "Function: _split_wrt_regions" << endl;
                py::print(e.what());
                continue;
            }
        }
    }

    return vec_nodes_split;
}

ReachNodePtr ReachableSet::update_propositions_with_region(ReachNodePtr const& node,
                                                           RegionPtr const& region, int const& step) {

    return semantic_model->obj_semantic_model_py.attr("update_propositions_with_region")(node, region, step)
            .cast<ReachNodePtr>();
}

vector<ReachNodePtr> ReachableSet::_split_wrt_intervals(int const& step, vector<ReachNodePtr> const& vec_nodes) {
    if (vec_nodes.empty()) {
        return {};
    }

    vector<ReachNodePtr> vec_nodes_split = {};
    vector<ReachNodePtr> vec_nodes_split_lon{};
    auto vec_intervals_lon = semantic_model->map_step_to_position_intervals[step]["lon"];
    auto vec_intervals_lat = semantic_model->map_step_to_position_intervals[step]["lat"];

    // longitudinal direction
    for (auto const& node: vec_nodes) {
        for (auto const& interval_lon: vec_intervals_lon) {
            if (interval_lon->intersects(node->p_lon_min(), node->p_lon_max())) {
                auto node_split = split_reach_node_wrt_interval(node, interval_lon, "lon");
                if (node_split) {
                    vec_nodes_split_lon.emplace_back(node_split);
                }
            }
        }
    }

    // lateral direction
    for (auto const& node: vec_nodes_split_lon) {
        for (auto const& interval_lat: vec_intervals_lat) {
            if (interval_lat->intersects(node->p_lat_min(), node->p_lat_max())) {
                auto node_split = split_reach_node_wrt_interval(node, interval_lat, "lat");
                if (node_split) {
                    vec_nodes_split.emplace_back(node_split);
                }
            }
        }
    }

    return vec_nodes_split;
}

vector<ReachNodePtr> ReachableSet::_discard_colliding_nodes(vector<ReachNodePtr> const& vec_nodes) {
    try {
        auto vec_nodes_keep =
                semantic_model->obj_semantic_model_py.attr("discard_colliding_nodes")(vec_nodes)
                        .cast<vector<ReachNodePtr>>();

        return vec_nodes_keep;
    }
    catch (py::error_already_set& e) {
        cout << "Function: _discard_colliding_nodes" << endl;
        py::print(e.what());

        return {};
    }
}

unordered_map<PropositionHolder, vector<ReachPolygonPtr>, PropositionHolder::HashFunction>
ReachableSet::_compute_collision_free_drivable_area(int const& step,
                                                    unordered_map<PropositionHolder, vector<ReachNodePtr>,
                                                            PropositionHolder::HashFunction> const&
                                                    map_propositions_to_drivable_area) {
    auto mode_repartition = config->reachable_set().mode_repartition;
    auto size_grid = config->reachable_set().size_grid;
    auto size_grid_2nd = config->reachable_set().size_grid_2nd;
    auto radius_terminal_split = config->reachable_set().radius_terminal_split;
    unordered_map<PropositionHolder, vector<ReachPolygonPtr>, PropositionHolder::HashFunction>
            map_proposition_holder_to_drivable_area{};

    // individually iterate through lists of propagated sets with different sets of propositions
    for (auto const& [proposition_holder, vec_propagated_sets]: map_propositions_to_drivable_area) {
        auto vec_rectangles_projected = project_propagated_sets_to_position_domain(vec_propagated_sets);

        vector<ReachPolygonPtr> drivable_area{};
        // repartition, then collision check
        if (mode_repartition == 1) {
            // create repartitioned rectangles from the projected base sets
            vec_rectangles_projected = create_repartitioned_rectangles(vec_rectangles_projected, size_grid);
            drivable_area = check_collision_and_split_rectangles(step, collision_checker,
                                                                 vec_rectangles_projected,
                                                                 radius_terminal_split,
                                                                 config->reachable_set().num_threads);
        }
            // collision check, then repartition
        else if (mode_repartition == 2) {
            auto vec_rectangles_collision_free = \
                        check_collision_and_split_rectangles(step, collision_checker,
                                                             vec_rectangles_projected,
                                                             radius_terminal_split,
                                                             config->reachable_set().num_threads);
            drivable_area = create_repartitioned_rectangles(vec_rectangles_collision_free, size_grid);
        }

            // repartition, collision check, then repartition again
        else if (mode_repartition == 3) {
            auto vec_rectangles_repartitioned = \
                        create_repartitioned_rectangles(vec_rectangles_projected, size_grid);

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

        map_proposition_holder_to_drivable_area[proposition_holder] = drivable_area;
    }

    return map_proposition_holder_to_drivable_area;

    // the following code also considers three-circle approximation
    //vector<ReachPolygonPtr> drivable_area_collision_free{};
    //// repartition, then collision check
    //if (config->reachable_set().mode_repartition == 1) {
    //    auto vec_rectangles_repartitioned = create_repartitioned_rectangles(
    //            vec_rectangles_projected, config->reachable_set().size_grid);
    //    if (config->reachable_set().mode_inflation != 3) {
    //        drivable_area_collision_free = check_collision_and_split_rectangles(
    //                step, collision_checker,
    //                vec_rectangles_repartitioned, config->reachable_set().radius_terminal_split,
    //                config->reachable_set().num_threads);
    //    } else {
    //        drivable_area_collision_free = check_collision_and_split_rectangles(
    //                step, collision_checker, vec_rectangles_repartitioned,
    //                config->reachable_set().radius_terminal_split,
    //                config->reachable_set().num_threads,
    //                config->vehicle().ego.circle_distance,
    //                *config->planning().CLCS,
    //                *config->reachable_set().lut_lon_enlargement,
    //                config->planning().reference_point);
    //    }
    //    // collision check, then repartition
    //} else if (config->reachable_set().mode_repartition == 2) {
    //    vector<ReachPolygonPtr> vec_rectangles_collision_free{};
    //    if (config->reachable_set().mode_inflation != 3) {
    //        vec_rectangles_collision_free = check_collision_and_split_rectangles(
    //                step, collision_checker,
    //                vec_rectangles_projected, config->reachable_set().radius_terminal_split,
    //                config->reachable_set().num_threads);
    //    } else {
    //        vec_rectangles_collision_free = check_collision_and_split_rectangles(
    //                step, collision_checker, vec_rectangles_projected,
    //                config->reachable_set().radius_terminal_split,
    //                config->reachable_set().num_threads,
    //                config->vehicle().ego.circle_distance,
    //                *config->planning().CLCS,
    //                *config->reachable_set().lut_lon_enlargement,
    //                config->planning().reference_point);
    //    }
    //    drivable_area_collision_free = create_repartitioned_rectangles(
    //            vec_rectangles_collision_free, config->reachable_set().size_grid);
    //    // repartition, collision check, then repartition again
    //} else if (config->reachable_set().mode_repartition == 3) {
    //    auto vec_rectangles_repartitioned = create_repartitioned_rectangles(
    //            vec_rectangles_projected, config->reachable_set().size_grid);
    //
    //    vector<ReachPolygonPtr> vec_rectangles_collision_free{};
    //    if (config->reachable_set().mode_inflation != 3) {
    //        vec_rectangles_collision_free = check_collision_and_split_rectangles(
    //                step, collision_checker,
    //                vec_rectangles_repartitioned, config->reachable_set().radius_terminal_split,
    //                config->reachable_set().num_threads);
    //    } else {
    //        vec_rectangles_collision_free = check_collision_and_split_rectangles(
    //                step, collision_checker, vec_rectangles_projected,
    //                config->reachable_set().radius_terminal_split,
    //                config->reachable_set().num_threads,
    //                config->vehicle().ego.circle_distance,
    //                *config->planning().CLCS,
    //                *config->reachable_set().lut_lon_enlargement,
    //                config->planning().reference_point);
    //    }
    //
    //    drivable_area_collision_free = create_repartitioned_rectangles(
    //            vec_rectangles_collision_free, config->reachable_set().size_grid);
    //
    //} else throw std::logic_error("Invalid mode for repartition.");
}

/// *Steps*:
/// 1. construct reach nodes from drivable area and the propagated base sets.
/// 2. update parent-child relationship of the nodes.
void ReachableSet::_compute_reachable_set_at_step(int const& step) {
    auto map_propositions_to_propagated_set = map_step_to_propositions_to_propagated_set[step];
    auto map_propositions_to_drivable_area = map_step_to_propositions_to_drivable_area[step];

    if (map_propositions_to_drivable_area.empty()) {
        map_step_to_propositions_to_reachable_set[step] = {};
        return;
    }

    auto num_threads = config->reachable_set().num_threads;
    unsigned long num_drivable_area = 0;
    for (auto const& [proposition_holder, drivable_area]: map_propositions_to_drivable_area) {
        num_drivable_area += drivable_area.size();
    }
    bool discard_small_node = (num_drivable_area > 1);

    unordered_map<PropositionHolder, vector<ReachNodePtr>, PropositionHolder::HashFunction>
            map_propositions_to_reachable_set{};
    for (auto const& [proposition_holder, drivable_area]: map_propositions_to_drivable_area) {
        auto propagated_set = map_propositions_to_propagated_set[proposition_holder];

        auto vec_nodes = construct_reach_nodes(drivable_area, propagated_set, num_threads);

        if (discard_small_node) {
            vec_nodes = discard_nodes_with_short_edge(vec_nodes, config->reachable_set().length_edge_node_min);
        }

        if (!vec_nodes.empty()) {
            auto reachable_set = connect_children_to_parents(step, vec_nodes, num_threads);
            map_propositions_to_reachable_set[proposition_holder] = reachable_set;
        }
    }
    map_step_to_propositions_to_reachable_set[step] = map_propositions_to_reachable_set;
}
//
///// Iterates through reachability graph backward in time, discards nodes that don't have a child node.
//void ReachableSet::prune_nodes_not_reaching_final_step() {
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
//        vector<ReachPolygonPtr> vec_drivable_area_updated{};
//        vector<ReachNodePtr> vec_reachable_set_updated{};
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

vector<ReachNodePtr> ReachableSet::_call_python_dummy(int const& step, vector<ReachNodePtr> const& vec_nodes) {
    vector<ReachNodePtr> vec_nodes_new{};
    for (auto const& node: vec_nodes) {
        vec_nodes_new.emplace_back(semantic_model->obj_semantic_model_py.attr("call_python_dummy")(step, node)
                                           .cast<ReachNodePtr>());
    }
    return vec_nodes_new;
}