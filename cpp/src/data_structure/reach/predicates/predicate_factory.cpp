#include "reach_semantic/data_structure/reach/predicates/predicate_factory.hpp"
#include "reach_semantic/data_structure/reach/predicates/py_predicate.hpp"

#include <regex>
#include <spdlog/spdlog.h>

using namespace semantic_reach;

PredicateFactory::PredicateFactory(std::shared_ptr<PredicateConfiguration> config) : config(std::move(config)) {
    // Check if there is an active Python interpreter, and if so import the module with Python predicates
    predicates_module =
        Py_IsInitialized() != 0
            ? std::optional{pybind11::module::import("commonroad_reach_semantic.data_structure.reach.predicates")}
            : std::nullopt;
}

std::unique_ptr<Predicate> PredicateFactory::predicate_from_proposition(const std::string &proposition,
                                                                        bool negated) const {
    std::smatch match;

    // in front of predicate
    if (std::regex_match(proposition, match, std::regex(R"(InFrontOf_V(\d+))"))) {
        size_t obstacle_id{static_cast<size_t>(std::stoi(match[1]))};
        return make_in_front_of_obstacle_predicate(negated, obstacle_id);
    }

    // behind predicate
    if (std::regex_match(proposition, match, std::regex(R"(Behind_V(\d+))"))) {
        size_t obstacle_id{static_cast<size_t>(std::stoi(match[1]))};
        return make_behind_obstacle_predicate(negated, obstacle_id);
    }

    // right of predicate
    if (std::regex_match(proposition, match, std::regex(R"(RightOf_V(\d+))"))) {
        size_t obstacle_id{static_cast<size_t>(std::stoi(match[1]))};
        return make_right_of_obstacle_predicate(negated, obstacle_id);
    }

    // left of predicate
    if (std::regex_match(proposition, match, std::regex(R"(LeftOf_V(\d+))"))) {
        size_t obstacle_id{static_cast<size_t>(std::stoi(match[1]))};
        return make_left_of_obstacle_predicate(negated, obstacle_id);
    }

    // safe distance predicate
    if (std::regex_match(proposition, match, std::regex(R"(SafeDistance_V(\d+))"))) {
        size_t obstacle_id{static_cast<size_t>(std::stoi(match[1]))};
        return make_safe_distance_prec_predicate(negated, obstacle_id);
    }

    // Fall back to Python predicates
    if (predicates_module.has_value()) {
        return std::make_unique<PyPredicate>(predicates_module.value().attr("from_proposition")(proposition, negated));
    } else {
        spdlog::warn("Python predicates are not available, cannot parse proposition: {}", proposition);
        throw std::invalid_argument("Unknown proposition: " + proposition);
    }
}

std::unique_ptr<InFrontOfObstaclePredicate>
PredicateFactory::make_in_front_of_obstacle_predicate(bool negated, size_t obstacle_id) const {
    return std::make_unique<InFrontOfObstaclePredicate>(negated, obstacle_id, config->ego_length);
}

std::unique_ptr<BehindObstaclePredicate> PredicateFactory::make_behind_obstacle_predicate(bool negated,
                                                                                          size_t obstacle_id) const {
    return std::make_unique<BehindObstaclePredicate>(negated, obstacle_id, config->ego_length);
}

std::unique_ptr<RightOfObstaclePredicate> PredicateFactory::make_right_of_obstacle_predicate(bool negated,
                                                                                             size_t obstacle_id) const {
    return std::make_unique<RightOfObstaclePredicate>(negated, obstacle_id, config->ego_width);
}

std::unique_ptr<LeftOfObstaclePredicate> PredicateFactory::make_left_of_obstacle_predicate(bool negated,
                                                                                           size_t obstacle_id) const {
    return std::make_unique<LeftOfObstaclePredicate>(negated, obstacle_id, config->ego_width);
}

std::unique_ptr<KeepSafeDistancePrecPredicate>
PredicateFactory::make_safe_distance_prec_predicate(bool negated, size_t obstacle_id) const {
    return std::make_unique<KeepSafeDistancePrecPredicate>(negated, obstacle_id, config->ego_length,
                                                           config->ego_reaction_time, config->ego_deceleration,
                                                           config->vehicle_deceleration);
}
