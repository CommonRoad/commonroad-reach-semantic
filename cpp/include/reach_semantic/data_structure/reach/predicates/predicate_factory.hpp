#pragma once

#include "reach_semantic/data_structure/reach/predicates/position/in_front_of_obstacle_predicate.hpp"
#include "reach_semantic/data_structure/reach/predicates/position/keeps_safe_distance_prec_predicate.hpp"
#include "reach_semantic/data_structure/reach/predicates/predicate.hpp"
#include "reach_semantic/data_structure/reach/predicates/predicate_config.hpp"

#include <pybind11/embed.h>

namespace semantic_reach {
/**
 * Factory class for predicates.
 */
class PredicateFactory {
  private:
    /**
     * Configuration that is used for all predicates.
     */
    std::shared_ptr<PredicateConfiguration> config;

    /**
     * Python module with predicates.
     *
     * Will be empty if no Python interpreter is available.
     */
    std::optional<pybind11::module_> predicates_module;

  public:
    /**
     * Constructor for predicate factory.
     *
     * @param config Configuration that is used for all predicates.
     */
    explicit PredicateFactory(std::shared_ptr<PredicateConfiguration> config);

    /**
     * Tries to parse a proposition into a predicate.
     *
     * @param proposition The proposition to parse.
     * @param negated Whether the proposition is negated.
     * @return The parsed predicate.
     * @throws std::invalid_argument if the proposition could not be parsed.
     */
    [[nodiscard]] std::unique_ptr<Predicate> predicate_from_proposition(const std::string &proposition,
                                                                        bool negated) const;

    /**
     * Create an in front of obstacle predicate using the factory's configuration.
     *
     * @param negated Whether the predicate is negated.
     * @param obstacle_id ID of the obstacle.
     * @return The created predicate.
     */
    [[nodiscard]] std::unique_ptr<InFrontOfObstaclePredicate>
    make_in_front_of_obstacle_predicate(bool negated, size_t obstacle_id) const;

    /**
     * Create a safe distance predicate using the factory's configuration.
     *
     * @param negated Whether the predicate is negated.
     * @param obstacle_id ID of the obstacle.
     * @return The created predicate.
     */
    [[nodiscard]] std::unique_ptr<KeepSafeDistancePrecPredicate>
    make_safe_distance_prec_predicate(bool negated, size_t obstacle_id) const;
};
} // namespace semantic_reach
