#include "reach_semantic/utility/reach_operation.hpp"
#include "reachset/utility/sweep_line.hpp"
#include "reachset/utility/shared_using.hpp"

#pragma clang diagnostic push
#pragma ide diagnostic ignored "UnusedLocalVariable"
using namespace semantic_reach;


SemanticReachNodePtr semantic_reach::split_reach_node_wrt_interval(SemanticReachNodePtr const& node, PositionIntervalPtr const& interval,
                                                  string const& direction) {
    auto node_split = node->clone();
    node_split->proposition_holder.add_propositions(interval->set_propositions, PropositionGroup::POSITION);

    try {
        if (direction == "lon") {
            node_split->polygon_lon->intersect_halfspace(1, 0, interval->p_max);
            node_split->polygon_lon->intersect_halfspace(-1, 0, -interval->p_min);

        } else if (direction == "lat") {
            node_split->polygon_lat->intersect_halfspace(1, 0, interval->p_max);
            node_split->polygon_lat->intersect_halfspace(-1, 0, -interval->p_min);

        } else {
            throw std::logic_error("Given direction is not valid.");
        }
    }
    catch (std::exception& e) {
        node_split = nullptr;
    }

    // check the validity of the split node
    if (not node_split or node_split->polygon_lon->empty() or node_split->polygon_lat->empty()) {
        node_split = nullptr;
    }

    return node_split;
}


vector<ReachPolygonPtr>
semantic_reach::project_propagated_sets_to_position_domain(vector<SemanticReachNodePtr> const& vec_propagated_set) {
    vector<ReachPolygonPtr> vec_rectangles_projected;
    vec_rectangles_projected.reserve(vec_propagated_set.size());

    for (auto const& propagated_set: vec_propagated_set) {
        vec_rectangles_projected.emplace_back(propagated_set->position_rectangle());
    }

    return vec_rectangles_projected;
}


/// *Steps*:
/// 1. examine the adjacency of drivable areas and the propagated sets. They are considered adjacent if they
///    overlap in the position domain.
/// 2. create a node from each drivable area and its adjacent propagated sets.
vector<SemanticReachNodePtr> semantic_reach::construct_reach_nodes(vector<ReachPolygonPtr> const& drivable_area,
                                                  vector<SemanticReachNodePtr> const& vec_propagated_set,
                                                  int const& num_threads) {

    vector<SemanticReachNodePtr> reachable_set;
    reachable_set.reserve(drivable_area.size());

    vector<ReachPolygonPtr> vec_rectangles_propagated_sets;
    vec_rectangles_propagated_sets.reserve(vec_propagated_set.size());

    for (auto const& propagated_set: vec_propagated_set) {
        vec_rectangles_propagated_sets.emplace_back(propagated_set->position_rectangle());
    }
    auto& vec_rectangles_drivable_area = drivable_area;
    auto map_rectangle_adjacency = create_adjacency_map(vec_rectangles_drivable_area,
                                                        vec_rectangles_propagated_sets);

#pragma omp parallel num_threads(num_threads) default(none) shared(drivable_area, map_rectangle_adjacency, \
vec_rectangles_drivable_area, vec_propagated_set, reachable_set)
    {
        vector<SemanticReachNodePtr> reachable_set_step_current_thread;
        reachable_set_step_current_thread.reserve(drivable_area.size());

#pragma omp for nowait
        for (int idx = 0; idx < map_rectangle_adjacency.size(); idx++) {
            auto it = map_rectangle_adjacency.begin();
            std::advance(it, idx);

            auto const idx_drivable_area = it->first;
            auto const vec_idx_propagated_sets_adjacent = it->second;
            auto const& rectangle_drivable_area = vec_rectangles_drivable_area[idx_drivable_area];

            auto reach_node = construct_reach_node(rectangle_drivable_area,
                                                   vec_propagated_set,
                                                   vec_idx_propagated_sets_adjacent);
            if (reach_node != nullptr) reachable_set_step_current_thread.emplace_back(reach_node);
        }
#pragma omp critical
        reachable_set.insert(reachable_set.end(),
                             std::make_move_iterator(reachable_set_step_current_thread.begin()),
                             std::make_move_iterator(reachable_set_step_current_thread.end()));
    }

    return reachable_set;
}

/// E.g.: {0 : [1, 2], 1 : [3, 4]} = rectangle_0 from 1st vector overlaps with rectangles_1 and _2 from the 2nd vector;
//  rectangle_1 from 1st vector overlaps with rectangles_3 and _4 from the 2nd vector.
unordered_map<int, vector<int>> semantic_reach::create_adjacency_map(vector<ReachPolygonPtr> const& vec_rectangles_a,
                                                            vector<ReachPolygonPtr> const& vec_rectangles_b) {
    unordered_map<int, vector<int>> map_idx_to_vec_idx;

    for (int idx_a = 0; idx_a < vec_rectangles_a.size(); idx_a++) {
        auto const& rectangle_a = vec_rectangles_a[idx_a];

        for (int idx_b = 0; idx_b < vec_rectangles_b.size(); idx_b++) {
            auto const& rectangle_b = vec_rectangles_b[idx_b];

            if (rectangle_a->intersects(rectangle_b)) { map_idx_to_vec_idx[idx_a].emplace_back(idx_b); }
        }
    }

    return map_idx_to_vec_idx;
}

/// Iterate through base sets that are adjacent to the drivable area, and cut the base sets down with position
/// constraints from the drivable area. A non-empty intersected polygon imply that it is a valid base set and is
/// considered as a parent of the rectangle (reachable from the node from which the base set is propagated).
SemanticReachNodePtr semantic_reach::construct_reach_node(ReachPolygonPtr const& rectangle_drivable_area,
                                         vector<SemanticReachNodePtr> const& vec_propagated_set,
                                         vector<int> const& vec_idx_propagated_sets_adjacent) {
    vector<SemanticReachNodePtr> vec_nodes_parent;
    vector<tuple<double, double>> vec_vertices_polygon_lon_new;
    vector<tuple<double, double>> vec_vertices_polygon_lat_new;
    auto proposition_holder = vec_propagated_set[0]->proposition_holder;

    // iterate through each of the adjacent base sets
    for (auto const& idx_propagated_set_adjacent: vec_idx_propagated_sets_adjacent) {
        auto const& propagated_set_adjacent = vec_propagated_set[idx_propagated_set_adjacent];
        auto polygon_lon = propagated_set_adjacent->polygon_lon->clone();
        auto polygon_lat = propagated_set_adjacent->polygon_lat->clone();
        // cut down to position range of the drivable area rectangle
        try {
            polygon_lon->intersect_halfspace(1, 0, rectangle_drivable_area->p_lon_max());
            polygon_lon->intersect_halfspace(-1, 0, -rectangle_drivable_area->p_lon_min());
            polygon_lat->intersect_halfspace(1, 0, rectangle_drivable_area->p_lat_max());
            polygon_lat->intersect_halfspace(-1, 0, -rectangle_drivable_area->p_lat_min());
        }
        catch (std::exception& e) {
            continue;
        }

        if (!polygon_lon->empty() and !polygon_lat->empty()) {
            // add to list if the intersected polygons are nonempty
            for (auto const& vertex: polygon_lon->vertices()) {
                vec_vertices_polygon_lon_new.emplace_back(vertex.x, vertex.y);
            }

            for (auto const& vertex: polygon_lat->vertices()) {
                vec_vertices_polygon_lat_new.emplace_back(vertex.x, vertex.y);
            }

            // the propagation of a node has only one source of propagation
            vec_nodes_parent.emplace_back(propagated_set_adjacent->vec_nodes_source[0]);
        }

    }

    if (not vec_vertices_polygon_lon_new.empty() and not vec_vertices_polygon_lat_new.empty()) {
        try {
            // if there is at least one valid base set
            auto polygon_lon_new = make_shared<ReachPolygon>(vec_vertices_polygon_lon_new);
            auto polygon_lat_new = make_shared<ReachPolygon>(vec_vertices_polygon_lat_new);
            polygon_lon_new->convexify();
            polygon_lat_new->convexify();

            // todo: check this later
            auto reach_node =
                    make_shared<SemanticReachNode>(-1, polygon_lon_new, polygon_lat_new, proposition_holder.clone());
            //reach_node->proposition_holder = proposition_holder->clone();
            reach_node->vec_nodes_source = vec_nodes_parent;

            return reach_node;
        }
        catch (std::exception& e) {
            return nullptr;
        }
    } else
        return nullptr;
}

vector<SemanticReachNodePtr> semantic_reach::discard_nodes_with_short_edge(vector<SemanticReachNodePtr> const& vec_nodes,
                                                          float const& length_edge_node_min) {
    vector<SemanticReachNodePtr> vec_nodes_to_keep{};
    for (auto const& node: vec_nodes) {
        auto length_lon = node->p_lon_max() - node->p_lon_min();
        auto length_lat = node->p_lat_max() - node->p_lat_min();

        if (length_lon >= length_edge_node_min and length_lat >= length_edge_node_min) {
            vec_nodes_to_keep.emplace_back(node);
        }
    }

    return vec_nodes_to_keep;
}

vector<SemanticReachNodePtr> semantic_reach::connect_children_to_parents(int const& step,
                                                        vector<SemanticReachNodePtr> const& vec_nodes,
                                                        int const& num_threads) {
    for (auto& node_child: vec_nodes) {
        node_child->step = step;
        // update parent-child relationship
        for (auto& node_parent: node_child->vec_nodes_source) {
            node_child->add_parent_node(node_parent);
            node_parent->add_child_node(node_child);
        }
    }

    return vec_nodes;
}