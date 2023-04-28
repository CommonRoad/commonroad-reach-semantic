#include <utility>
#include "reach_semantic/data_structure/reach/semantic_reach_node.hpp"
#include "reachset/utility/shared_using.hpp"

using namespace semantic_reach;

int SemanticReachNode::cnt_id = 0;

SemanticReachNode::SemanticReachNode(int const& step, reach::ReachPolygonPtr polygon_lon, reach::ReachPolygonPtr polygon_lat,
                     PropositionHolder proposition_holder,
                     std::vector<std::shared_ptr<SemanticReachNode>> const& vec_nodes_source) :
        step(step), polygon_lon(std::move(polygon_lon)), polygon_lat(std::move(polygon_lat)),
        proposition_holder(std::move(proposition_holder)), vec_nodes_source(vec_nodes_source) {
    id = SemanticReachNode::cnt_id++;
}

bool SemanticReachNode::add_parent_node(SemanticReachNodePtr const& node_parent) {
    if (std::none_of(vec_nodes_parent.cbegin(), vec_nodes_parent.cend(),
                     [&](auto const& node) { return node == node_parent; })) {
        vec_nodes_parent.emplace_back(node_parent);
        return true;
    }

    return false;
}


bool SemanticReachNode::remove_parent_node(SemanticReachNodePtr const& node_parent) {
    auto it_end = std::remove(vec_nodes_parent.begin(), vec_nodes_parent.end(), node_parent);
    if (it_end != vec_nodes_parent.end()) {
        vec_nodes_parent.erase(it_end, vec_nodes_parent.end());
        return true;
    }

    return false;
}

bool SemanticReachNode::add_child_node(SemanticReachNodePtr const& node_child) {
    if (std::none_of(vec_nodes_child.cbegin(),
                     vec_nodes_child.cend(),
                     [&](auto const& node) { return node == node_child; })) {
        vec_nodes_child.emplace_back(node_child);
        return true;
    }

    return false;
}

bool SemanticReachNode::remove_child_node(SemanticReachNodePtr const& node_child) {
    auto it_end = std::remove(vec_nodes_child.begin(), vec_nodes_child.end(), node_child);
    if (it_end != vec_nodes_child.end()) {
        vec_nodes_child.erase(it_end, vec_nodes_child.end());
        return true;
    }

    return false;
}