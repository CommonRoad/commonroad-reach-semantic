#include "reach_semantic/data_structure/reach/predicates/cpp_predicate.hpp"

using namespace semantic_reach;
using geometry::CurvilinearCoordinateSystem;

CppPredicate::CppPredicate(bool negated, bool needs_lanelets) : Predicate(needs_lanelets), negated(negated) {}

std::vector<reach::ReachNodePtr> CppPredicate::restrict_reach_node(
    int step, const reach::ReachNodePtr &reach_node, const semantic_reach::SemanticModelPtr &semantic_model,
    const std::shared_ptr<World> &world, const std::shared_ptr<CurvilinearCoordinateSystem> &ego_ccs) const {
    if (needs_lanelets) {
        throw std::runtime_error("Predicate needs lanelets, but none were provided.");
    }
    return restrict_reach_node(step, reach_node, semantic_model, world, ego_ccs, {});
}

std::vector<reach::ReachNodePtr> CppPredicate::restrict_reach_node(
    int step, const reach::ReachNodePtr &reach_node, const semantic_reach::SemanticModelPtr &semantic_model,
    const std::shared_ptr<World> &world, const std::shared_ptr<CurvilinearCoordinateSystem> &ego_ccs,
    const std::set<int> &node_lanelet_ids) const {
    auto restricted_nodes = negated ? _restrict_reach_node_forbidden(step, reach_node, world, ego_ccs)
                                    : _restrict_reach_node_mandatory(step, reach_node, world, ego_ccs);

    restricted_nodes.erase(std::remove_if(restricted_nodes.begin(), restricted_nodes.end(),
                                          [](const reach::ReachNodePtr &node) { return node->is_empty(); }),
                           restricted_nodes.end());

    return restricted_nodes;
}
