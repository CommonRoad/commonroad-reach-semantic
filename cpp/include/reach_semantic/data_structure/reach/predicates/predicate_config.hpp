#pragma once

#include "reach_semantic/data_structure/semantic_configuration.hpp"

#include <commonroad_cpp/predicates/predicate_config.h>

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
         * Construct predicate configuration from semantic configuration.
         *
         * @param config The semantic configuration.
         */
        PredicateConfiguration(const SemanticConfiguration &config);
    };
}
