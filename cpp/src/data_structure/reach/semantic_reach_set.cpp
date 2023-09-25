#include "reach_semantic/data_structure/reach/semantic_reach_set.hpp"

#include "reach_semantic/utility/reach_operation.hpp"
#include "reachset/utility/reach_operation.hpp"
#include "reachset/utility/shared_using.hpp"
#include <utility>

using namespace semantic_reach;

SemanticReachableSet::SemanticReachableSet(SemanticConfigurationPtr config, CollisionCheckerPtr collision_checker,
                                           SemanticModelPtr semantic_model,
                                           TrafficRuleInterfacePtr traffic_rule_interface)
    : config(std::move(config)), collision_checker(std::move(collision_checker)),
      semantic_model(std::move(semantic_model)), rule_interface(std::move(traffic_rule_interface)) {
    _initialize_zero_state_polygons();

    step_start = this->config->planning().step_start;
    step_end = step_start + this->config->planning().steps_computation;
}

/// @note Computation of the reachable set of an LTI system requires the zero-state response of the system.
void SemanticReachableSet::_initialize_zero_state_polygons() {
    polygon_zero_state_lon = create_zero_state_polygon(config->planning().dt, config->vehicle().ego.a_lon_min,
                                                       config->vehicle().ego.a_lon_max);

    polygon_zero_state_lat = create_zero_state_polygon(config->planning().dt, config->vehicle().ego.a_lat_min,
                                                       config->vehicle().ego.a_lat_max);
}

std::vector<reach::ReachNodePtr> SemanticReachableSet::_construct_initial_reachable_sets() {
    // initial drivable area
    auto tuple_vertices = generate_tuple_vertices_position_rectangle_initial(config);

    // initial reachable set
    auto [tuple_vertices_polygon_lon, tuple_vertices_polygon_lat] = generate_tuples_vertices_polygons_initial(config);
    auto polygon_lon = make_shared<reach::ReachPolygon>(tuple_vertices_polygon_lon);
    auto polygon_lat = make_shared<reach::ReachPolygon>(tuple_vertices_polygon_lat);
    auto node = std::make_shared<reach::ReachNode>(step_start, polygon_lon, polygon_lat);
    // set source propagation to vector containing nullptr as first element, as the vector is expected to be non-empty
    // by subsequent methods
    node->vec_nodes_source = {nullptr};
    return {node};
}

void SemanticReachableSet::compute(int step_start, int step_end) {
    if (step_start == 0)
        step_start = this->step_start + 1;
    if (step_end == 0)
        step_end = this->step_end;

    for (auto step = step_start; step < step_end + 1; step++) {
        _compute_drivable_area_at_step(step);
        _compute_reachable_set_at_step(step);
        _vec_steps_computed.emplace_back(step);
    }

    // if (step_start != step_end and config->reachable_set().prune_nodes) {
    //     prune_nodes_not_reaching_final_step();
    // }
}

vector<reach::ReachNodePtr>
SemanticReachableSet::_propagate_reachable_set(vector<reach::ReachNodePtr> const &vec_nodes) {
    vector<reach::ReachNodePtr> vec_base_sets_propagated;
    vec_base_sets_propagated.reserve(vec_nodes.size());

#pragma omp parallel num_threads(config->reachable_set().num_threads) default(none)                                    \
    shared(vec_nodes, vec_base_sets_propagated)
    {
        vector<reach::ReachNodePtr> vec_base_sets_propagated_thread;
        vec_base_sets_propagated_thread.reserve(vec_nodes.size());

#pragma omp for nowait
        for (auto const &node : vec_nodes) {
            try {
                auto polygon_lon_propagated =
                    propagate_polygon(node->polygon_lon, polygon_zero_state_lon, config->planning().dt,
                                      config->vehicle().ego.v_lon_min, config->vehicle().ego.v_lon_max);

                auto polygon_lat_propagated =
                    propagate_polygon(node->polygon_lat, polygon_zero_state_lat, config->planning().dt,
                                      config->vehicle().ego.v_lat_min, config->vehicle().ego.v_lat_max);

                auto propagated_set =
                    make_shared<reach::ReachNode>(node->step, polygon_lon_propagated, polygon_lat_propagated);
                propagated_set->vec_nodes_source.emplace_back(node);
                vec_base_sets_propagated_thread.emplace_back(propagated_set);
            } catch (std::exception &e) {
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
SemanticReachableSet::_collision_check_and_repartition(std::vector<reach::ReachPolygonPtr> rectangles,
                                                       int const &step) {
    auto mode_repartition = config->reachable_set().mode_repartition;
    auto size_grid = config->reachable_set().size_grid;
    auto size_grid_2nd = config->reachable_set().size_grid_2nd;
    auto radius_terminal_split = config->reachable_set().radius_terminal_split;

    vector<reach::ReachPolygonPtr> drivable_area{};
    // repartition, then collision check
    if (mode_repartition == 1) {
        // create repartitioned rectangles from the projected base sets
        auto vec_rectangles_repartitioned = create_repartitioned_rectangles(rectangles, size_grid);
        drivable_area =
            check_collision_and_split_rectangles(step, collision_checker, vec_rectangles_repartitioned,
                                                 radius_terminal_split, config->reachable_set().num_threads);
    }

    // collision check, then repartition
    else if (mode_repartition == 2) {
        auto vec_rectangles_collision_free = check_collision_and_split_rectangles(
            step, collision_checker, rectangles, radius_terminal_split, config->reachable_set().num_threads);
        drivable_area = create_repartitioned_rectangles(vec_rectangles_collision_free, size_grid);
    }

    // repartition, collision check, then repartition again
    else if (mode_repartition == 3) {
        auto vec_rectangles_repartitioned = create_repartitioned_rectangles(rectangles, size_grid);

        auto vec_rectangles_collision_free =
            check_collision_and_split_rectangles(step, collision_checker, vec_rectangles_repartitioned,
                                                 radius_terminal_split, config->reachable_set().num_threads);

        drivable_area = create_repartitioned_rectangles(vec_rectangles_collision_free, size_grid_2nd);
    } else {
        throw(std::logic_error("Invalid mode for repartition."));
    }

    return drivable_area;
}

/// Iterates through reachability graph backward in time, discards nodes that don't have a child node.
void SemanticReachableSet::prune_nodes_not_reaching_final_step() {
    auto cnt_nodes_before_pruning = reachable_set_at_step(step_end).size();
    auto cnt_nodes_after_pruning = cnt_nodes_before_pruning;

    for (auto step = step_end - 1; step > step_start - 1; step--) {
        auto vec_nodes = reachable_set_at_step(step);
        cnt_nodes_before_pruning += vec_nodes.size();

        vector<int> vec_idx_nodes_to_be_deleted{};
        for (int idx_node = 0; idx_node < vec_nodes.size(); idx_node++) {
            auto node = vec_nodes[idx_node];
            // discard the node if it has no child node
            if (node->vec_nodes_child().empty()) {
                vec_idx_nodes_to_be_deleted.push_back(idx_node);
                // iterate through its parent nodes and disconnect them
                for (auto const &node_parent : node->vec_nodes_parent()) {
                    node_parent->remove_child_node(node);
                }
            }
        }
        // discard nodes without a child
        vector<reach::ReachPolygonPtr> vec_drivable_area_updated{};
        vector<reach::ReachNodePtr> vec_reachable_set_updated{};
        for (int idx_node = 0; idx_node < vec_nodes.size(); idx_node++) {
            auto result = std::find(vec_idx_nodes_to_be_deleted.begin(), vec_idx_nodes_to_be_deleted.end(), idx_node) !=
                          vec_idx_nodes_to_be_deleted.end();

            if (not result) {
                auto node = vec_nodes[idx_node];
                vec_drivable_area_updated.emplace_back(node->position_rectangle());
                vec_reachable_set_updated.emplace_back(node);
            }
        }
        // update drivable area and reachable set dictionaries
        map_step_to_drivable_area[step] = vec_drivable_area_updated;
        map_step_to_reachable_set[step] = vec_reachable_set_updated;

        cnt_nodes_after_pruning += map_step_to_reachable_set[step].size();
    }

    _pruned = true;

    std::cout << "\t#Nodes before pruning: \t" << cnt_nodes_before_pruning << endl;
    std::cout << "\t#Nodes after pruning: \t" << cnt_nodes_after_pruning << endl;
}
