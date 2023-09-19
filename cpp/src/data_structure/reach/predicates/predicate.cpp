#include "reach_semantic/data_structure/reach/predicates/predicate.hpp"
#include "reach_semantic/data_structure/reach/predicates/py_predicate.hpp"
#include "reach_semantic/data_structure/reach/predicates/in_front_of_obstacle_predicate.hpp"

using namespace semantic_reach;


Predicate::Predicate(bool negated, bool needs_lanelets) : negated(negated), needs_lanelets(needs_lanelets) {}

std::vector<reach::ReachNodePtr>
Predicate::restrict_reach_node(int step, const reach::ReachNodePtr &reach_node,
                               const semantic_reach::SemanticModelPtr &semantic_model,
                               const std::shared_ptr<World> &world,
                               const std::shared_ptr<CurvilinearCoordinateSystem> &ego_ccs) const {
    if (needs_lanelets) {
        throw std::runtime_error("Predicate needs lanelets, but none were provided.");
    }
    return restrict_reach_node(step, reach_node, semantic_model, world, ego_ccs, {});
}

std::vector<reach::ReachNodePtr>
Predicate::restrict_reach_node(int step, const reach::ReachNodePtr &reach_node,
                               const semantic_reach::SemanticModelPtr &semantic_model,
                               const std::shared_ptr<World> &world,
                               const std::shared_ptr<CurvilinearCoordinateSystem> &ego_ccs,
                               const std::set<int> &node_lanelet_ids) const {
    auto restricted_nodes = negated ?
                            _restrict_reach_node_forbidden(step, reach_node, world, ego_ccs) :
                            _restrict_reach_node_mandatory(step, reach_node, world, ego_ccs);

    restricted_nodes.erase(
            std::remove_if(restricted_nodes.begin(), restricted_nodes.end(), [](const reach::ReachNodePtr &node) {
                return node->is_empty();
            }), restricted_nodes.end());

    return restricted_nodes;
}


std::unique_ptr<Predicate>
Predicate::from_proposition(const std::string &proposition, bool is_negated) {
    auto in_front_of = InFrontOfObstaclePredicate::try_from_proposition(proposition, is_negated);
    if (in_front_of.has_value()) {
        return std::move(in_front_of.value());
    }

    // Fall back to Python predicates
    auto predicates_module = pybind11::module::import("commonroad_reach_semantic.data_structure.reach.predicates");
    auto obj_predicate_py = predicates_module.attr("from_proposition")(proposition, is_negated);
    return std::make_unique<PyPredicate>(is_negated, obj_predicate_py);
}
