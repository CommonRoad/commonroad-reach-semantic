#include "reach_semantic/data_structure/reach/predicates/predicate.hpp"

using namespace semantic_reach;

Predicate::Predicate(pybind11::object obj_predicate_py) : obj_predicate_py(obj_predicate_py) {
    needs_lanelets = this->obj_predicate_py.attr("needs_lanelets").cast<bool>();
}

std::vector<reach::ReachNodePtr>
Predicate::restrict_reach_node(int step, const reach::ReachNodePtr &reach_nodes,
                               const semantic_reach::SemanticModelPtr &semantic_model) const {
    if (needs_lanelets) {
        throw std::runtime_error("Predicate needs lanelets, but none were provided.");
    }
    pybind11::list reach_node_list_py =
        obj_predicate_py.attr("restrict_reach_node")(step, reach_nodes, semantic_model->obj_semantic_model_py);
    std::vector<reach::ReachNodePtr> vec_reach_nodes{};
    vec_reach_nodes.reserve(reach_node_list_py.size());
    for (auto &reach_node_py : reach_node_list_py) {
        vec_reach_nodes.emplace_back(reach_node_py.cast<reach::ReachNodePtr>());
    }
    return vec_reach_nodes;
}

std::vector<reach::ReachNodePtr> Predicate::restrict_reach_node(int step, const reach::ReachNodePtr &reach_nodes,
                                                                const semantic_reach::SemanticModelPtr &semantic_model,
                                                                const std::set<int> &node_lanelet_ids) const {
    pybind11::list reach_node_list_py = obj_predicate_py.attr("restrict_reach_node")(
        step, reach_nodes, semantic_model->obj_semantic_model_py, node_lanelet_ids);
    std::vector<reach::ReachNodePtr> vec_reach_nodes{};
    vec_reach_nodes.reserve(reach_node_list_py.size());
    for (auto &reach_node_py : reach_node_list_py) {
        vec_reach_nodes.emplace_back(reach_node_py.cast<reach::ReachNodePtr>());
    }
    return vec_reach_nodes;
}

Predicate Predicate::from_proposition(const std::string &proposition, bool negated) {
    auto predicates_module = pybind11::module::import("commonroad_reach_semantic.data_structure.reach.predicates");
    auto obj_predicate_py = predicates_module.attr("from_proposition")(proposition, negated);
    return {obj_predicate_py};
}
