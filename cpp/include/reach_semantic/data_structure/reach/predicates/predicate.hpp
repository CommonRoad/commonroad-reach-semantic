#pragma once

#include <pybind11/embed.h>

#include "reachset/data_structure/reach/reach_node.hpp"
#include "reach_semantic/data_structure/semantic_model.hpp"

namespace semantic_reach {
    class Predicate {
    private:
        pybind11::object obj_predicate_py;

    public:
        bool needs_lanelets;

        Predicate(pybind11::object obj_predicate_py);

        static Predicate from_proposition(const std::string &proposition, bool negated);

        std::vector<reach::ReachNodePtr>
        restrict_reach_node(int step, const reach::ReachNodePtr &reach_nodes, const semantic_reach::SemanticModelPtr &semantic_model) const;

        std::vector<reach::ReachNodePtr>
        restrict_reach_node(int step, const reach::ReachNodePtr &reach_nodes, const semantic_reach::SemanticModelPtr &semantic_model,
                            const std::set<int> &node_lanelet_ids) const;
    };
}
