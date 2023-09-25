#include "reach_semantic/utility/reach_operation.hpp"
#include "reachset/utility/shared_using.hpp"
#include "reachset/utility/sweep_line.hpp"

#pragma clang diagnostic push
#pragma ide diagnostic ignored "UnusedLocalVariable"
using namespace semantic_reach;

reach::ReachNodePtr semantic_reach::split_reach_node_wrt_interval(reach::ReachNodePtr const &node,
                                                                  PositionIntervalPtr const &interval,
                                                                  string const &direction) {
    auto node_split = node->clone();

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
    } catch (std::exception &e) {
        node_split = nullptr;
    }

    // check the validity of the split node
    if (not node_split or node_split->polygon_lon->empty() or node_split->polygon_lat->empty()) {
        node_split = nullptr;
    }

    return node_split;
}

vector<reach::ReachNodePtr> semantic_reach::discard_nodes_with_short_edge(const vector<reach::ReachNodePtr> &vec_nodes,
                                                                          const double &length_edge_node_min) {
    vector<reach::ReachNodePtr> vec_nodes_to_keep{};
    for (auto const &node : vec_nodes) {
        auto length_lon = node->p_lon_max() - node->p_lon_min();
        auto length_lat = node->p_lat_max() - node->p_lat_min();

        if (length_lon >= length_edge_node_min and length_lat >= length_edge_node_min) {
            vec_nodes_to_keep.emplace_back(node);
        }
    }

    return vec_nodes_to_keep;
}
