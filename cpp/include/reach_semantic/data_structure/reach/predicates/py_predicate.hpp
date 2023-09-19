#pragma once

#include <pybind11/embed.h>

#include "reach_semantic/data_structure/reach/predicates/predicate.hpp"

namespace semantic_reach {
    class PyPredicate : public Predicate {
    private:
        pybind11::object obj_predicate_py;

        [[nodiscard]] std::vector<reach::ReachNodePtr>
        _restrict_reach_node_mandatory(int step, const reach::ReachNodePtr &reach_node, const std::shared_ptr<World> &world,
                                       const std::shared_ptr<CurvilinearCoordinateSystem> &ego_ccs) const override;

        [[nodiscard]] std::vector<reach::ReachNodePtr>
        _restrict_reach_node_forbidden(int step, const reach::ReachNodePtr &reach_node, const std::shared_ptr<World> &world,
                                       const std::shared_ptr<CurvilinearCoordinateSystem> &ego_ccs) const override;

    public:
        explicit PyPredicate(bool negated, pybind11::object obj_predicate_py);

        [[nodiscard]] std::vector<reach::ReachNodePtr>
        restrict_reach_node(int step, const reach::ReachNodePtr &reach_node,
                            const semantic_reach::SemanticModelPtr &semantic_model,
                            const std::shared_ptr<World> &world,
                            const std::shared_ptr<CurvilinearCoordinateSystem> &ego_ccs) const override;

        [[nodiscard]] std::vector<reach::ReachNodePtr>
        restrict_reach_node(int step, const reach::ReachNodePtr &reach_node,
                            const semantic_reach::SemanticModelPtr &semantic_model,
                            const std::shared_ptr<World> &world,
                            const std::shared_ptr<CurvilinearCoordinateSystem> &ego_ccs,
                            const std::set<int> &node_lanelet_ids) const override;
    };
}
