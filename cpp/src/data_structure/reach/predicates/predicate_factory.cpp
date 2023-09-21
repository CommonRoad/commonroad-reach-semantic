#include "reach_semantic/data_structure/reach/predicates/predicate_factory.hpp"
#include "reach_semantic/data_structure/reach/predicates/py_predicate.hpp"

#include "reach_semantic/data_structure/reach/predicates/position/in_front_of_obstacle_predicate.hpp"

#include <spdlog/spdlog.h>

using namespace semantic_reach;

PredicateFactory::PredicateFactory(std::shared_ptr<PredicateConfiguration> config) : config(std::move(config)) {
    // Check if there is an active Python interpreter, and if so import the module with Python predicates
    predicates_module =
        Py_IsInitialized()
            ? std::optional{pybind11::module::import("commonroad_reach_semantic.data_structure.reach.predicates")}
            : std::nullopt;
}

std::unique_ptr<Predicate> PredicateFactory::predicate_from_proposition(const std::string &proposition, bool negated) const {
    auto in_front_of = InFrontOfObstaclePredicate::try_from_proposition(proposition, config, negated);
    if (in_front_of.has_value()) {
        return std::move(in_front_of.value());
    }

    // Fall back to Python predicates
    if (predicates_module.has_value()) {
        return std::make_unique<PyPredicate>(predicates_module.value().attr("from_proposition")(proposition, negated));
    } else {
        spdlog::warn("Python predicates are not available, cannot parse proposition: {}", proposition);
        throw std::invalid_argument("Unknown proposition: " + proposition);
    }
}
