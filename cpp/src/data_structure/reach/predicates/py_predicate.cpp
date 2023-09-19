#include "reach_semantic/data_structure/reach/predicates/py_predicate.hpp"

using namespace semantic_reach;

PyPredicate::PyPredicate(bool negated, pybind11::object obj_predicate_py) :
        Predicate(negated, obj_predicate_py.attr("needs_lanelets").cast<bool>()),
        obj_predicate_py(obj_predicate_py) {}

std::vector<reach::ReachNodePtr> PyPredicate::restrict_reach_node(int step, const reach::ReachNodePtr &reach_node,
                                                                  const semantic_reach::SemanticModelPtr &semantic_model,
                                                                  const std::shared_ptr<World> &world,
                                                                  const std::shared_ptr<CurvilinearCoordinateSystem> &ego_ccs) const {
    if (needs_lanelets) {
        throw std::runtime_error("PyPredicate needs lanelets, but none were provided.");
    }
    return restrict_reach_node(step, reach_node, semantic_model, world, ego_ccs, {});
}

std::vector<reach::ReachNodePtr> PyPredicate::restrict_reach_node(int step, const reach::ReachNodePtr &reach_node,
                                                                  const semantic_reach::SemanticModelPtr &semantic_model,
                                                                  const std::shared_ptr<World> &world,
                                                                  const std::shared_ptr<CurvilinearCoordinateSystem> &ego_ccs,
                                                                  const std::set<int> &node_lanelet_ids) const {
    pybind11::list reach_node_list_py = obj_predicate_py.attr("restrict_reach_node")(step, reach_node,
                                                                                     semantic_model->obj_semantic_model_py,
                                                                                     node_lanelet_ids);
    std::vector<reach::ReachNodePtr> vec_reach_nodes{};
    vec_reach_nodes.reserve(reach_node_list_py.size());
    for (auto &reach_node_py: reach_node_list_py) {
        vec_reach_nodes.emplace_back(reach_node_py.cast<reach::ReachNodePtr>());
    }
    return vec_reach_nodes;
}

std::vector<reach::ReachNodePtr>
PyPredicate::_restrict_reach_node_mandatory(int step, const reach::ReachNodePtr &reach_node,
                                            const std::shared_ptr<World> &world,
                                            const std::shared_ptr<CurvilinearCoordinateSystem> &ego_ccs) const {
    throw std::runtime_error("PyPredicate does not support mandatory restrictions.");
}

std::vector<reach::ReachNodePtr>
PyPredicate::_restrict_reach_node_forbidden(int step, const reach::ReachNodePtr &reach_node,
                                            const std::shared_ptr<World> &world,
                                            const std::shared_ptr<CurvilinearCoordinateSystem> &ego_ccs) const {
    throw std::runtime_error("PyPredicate does not support forbidden restrictions.");
}
