#pragma once

#include "reach_semantic/data_structure/reach/predicates/predicate_config.hpp"
#include "reach_semantic/data_structure/reach/predicates/predicate.hpp"

#include <pybind11/embed.h>

namespace semantic_reach {
    class PredicateFactory {
    private:
        std::shared_ptr<PredicateConfiguration> config;
        std::optional<pybind11::module_> predicates_module;
    public:
        explicit PredicateFactory(std::shared_ptr<PredicateConfiguration> config);

        [[nodiscard]] std::unique_ptr<Predicate>
        predicate_from_proposition(const std::string &proposition, bool negated) const;
    };
}
