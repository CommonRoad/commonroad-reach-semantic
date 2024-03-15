#pragma once

#include "collision/collision_checker.h"
#include "geometry/curvilinear_coordinate_system.h"
#include "reach_semantic/data_structure/position_interval.hpp"
#include "reach_semantic/data_structure/proposition_holder.hpp"
#include "reach_semantic/data_structure/semantic_configuration.hpp"
#include "reachset/data_structure/reach/reach_node.hpp"
#include "reachset/data_structure/reach/reach_polygon.hpp"
#include "shared_include.hpp"

using CollisionCheckerPtr = collision::CollisionCheckerPtr;
using RectangleAABBPtr = collision::RectangleAABBPtr;

namespace semantic_reach {
using namespace reach;

reach::ReachNodePtr split_reach_node_wrt_interval(reach::ReachNodePtr const &node, PositionIntervalPtr const &interval,
                                                  string const &direction);

/// Discards nodes with an edge shorter than the specified length.
std::vector<reach::ReachNodePtr> discard_nodes_with_short_edge(const vector<reach::ReachNodePtr> &vec_nodes,
                                                               const double &length_edge_node_min);

} // namespace semantic_reach
