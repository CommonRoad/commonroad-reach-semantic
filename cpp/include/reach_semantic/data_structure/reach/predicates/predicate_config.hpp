#pragma once

#include "reach_semantic/data_structure/semantic_configuration.hpp"

#include <commonroad_cpp/predicates/predicate_parameter_collection.h>

namespace semantic_reach {
/**
 * Configuration for the predicates.
 *
 * Includes parameters for traffic rules and information about the ego vehicle.
 */
struct PredicateConfiguration {
    /**
     * Parameters for traffic rules.
     */
    PredicateParameters traffic_rule_params{};

    /**
     * Length of the ego vehicle.
     */
    double ego_length{4.5};

    /**
     * Width of the ego vehicle.
     */
    double ego_width{2.0};

    /**
     * Reaction time of the ego vehicle.
     */
    double ego_reaction_time{0.3};

    /**
     * Maximal feasible deceleration of the ego vehicle.
     */
    double ego_deceleration{-10.0};

    /**
     * Assumed maximal feasible deceleration of other vehicles.
     */
    double vehicle_deceleration{-10.5};

    /**
     * Construct predicate configuration with all default values.
     */
    PredicateConfiguration() = default;

    /**
     * Construct predicate configuration from semantic configuration.
     *
     * @param config The semantic configuration.
     */
    explicit PredicateConfiguration(const SemanticConfiguration &config);
};
} // namespace semantic_reach
