#include "reach_semantic/data_structure/reach/predicates/predicate.hpp"
#include "reach_semantic/data_structure/reach/predicates/py_predicate.hpp"
#include "reach_semantic/data_structure/reach/predicates/position/in_front_of_obstacle_predicate.hpp"

using namespace semantic_reach;


Predicate::Predicate(bool needs_lanelets) : needs_lanelets(needs_lanelets) {}

std::unique_ptr<Predicate>
Predicate::from_proposition(const std::string &proposition, bool is_negated) {
    auto in_front_of = InFrontOfObstaclePredicate::try_from_proposition(proposition, is_negated);
    if (in_front_of.has_value()) {
        return std::move(in_front_of.value());
    }

    // Fall back to Python predicates
    auto predicates_module = pybind11::module::import("commonroad_reach_semantic.data_structure.reach.predicates");
    return std::make_unique<PyPredicate>(predicates_module.attr("from_proposition")(proposition, is_negated));
}
