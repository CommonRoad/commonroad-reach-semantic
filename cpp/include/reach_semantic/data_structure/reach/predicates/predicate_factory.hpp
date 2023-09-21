#pragma once

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
};
} // namespace semantic_reach
