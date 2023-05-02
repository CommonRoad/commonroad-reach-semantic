#pragma once

#include "shared_include.hpp"
#include "reachset/data_structure/reach/reach_polygon.hpp"
#include "reach_semantic/data_structure/reach/semantic_reach_node.hpp"
#include "reach_semantic/data_structure/semantic_configuration.hpp"
#include "collision/collision_checker.h"
#include "geometry/curvilinear_coordinate_system.h"
#include "reach_semantic/data_structure/position_interval.hpp"

using CollisionCheckerPtr = collision::CollisionCheckerPtr;
using RectangleAABBPtr = collision::RectangleAABBPtr;

namespace semantic_reach {
    using namespace reach;

    SemanticReachNodePtr
    split_reach_node_wrt_interval(SemanticReachNodePtr const &node, PositionIntervalPtr const &interval,
                                  string const &direction);

    /// Returns a list of rectangles in the position domain.
    std::vector<ReachPolygonPtr>
    project_propagated_sets_to_position_domain(std::vector<SemanticReachNodePtr> const &vec_propagated_set);

    /// Constructs nodes of the reachability graph.
    /// The nodes are constructed by cutting down the propagated base sets to the drivable area to determine the reachable
    /// positions and velocities.
    std::vector<SemanticReachNodePtr> construct_reach_nodes(std::vector<ReachPolygonPtr> const &drivable_area,
                                                            std::vector<SemanticReachNodePtr> const &vec_propagated_set,
                                                            int const &num_threads);

    /// Creates a map indicating the adjacency (overlapping) status of two lists of rectangles.
    std::unordered_map<int, vector<int>> create_adjacency_map(std::vector<ReachPolygonPtr> const &vec_rectangles_a,
                                                              std::vector<ReachPolygonPtr> const &vec_rectangles_b);

    /// Returns a reach node constructed from the propagated sets.
    SemanticReachNodePtr construct_reach_node(ReachPolygonPtr const &rectangle_drivable_area,
                                              std::vector<SemanticReachNodePtr> const &vec_propagated_set,
                                              std::vector<int> const &vec_idx_propagated_sets_adjacent);

    /// Discards nodes with an edge shorter than the specified length.
    std::vector<SemanticReachNodePtr> discard_nodes_with_short_edge(std::vector<SemanticReachNodePtr> const &vec_nodes,
                                                                    float const &length_edge_node_min);

    /// Connects child reach nodes to their parent nodes.
    std::vector<SemanticReachNodePtr> connect_children_to_parents(int const &step,
                                                                  std::vector<SemanticReachNodePtr> const &vec_nodes,
                                                                  int const &num_threads);
}
