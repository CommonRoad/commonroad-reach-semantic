#pragma once

#include <spot/tl/formula.hh>

namespace util_spot {
    /// Return all conjuncts of a spot formula.
    std::vector<spot::formula> conjuncts(const spot::formula &formula);

    /// Return all disjuncts of a spot formula.
    std::vector<spot::formula> disjuncts(const spot::formula &formula);

    /// Extract the atomic proposition from a (negated) literal.
    /// @param literal Formula that is either a literal or a negated literal
    /// @returns Name of the atomic proposition and whether it is negated or not
    /// @throws std::invalid_argument if the formula is not a literal or a negated literal
    std::pair<std::string, bool> extract_atomic_proposition(const spot::formula &literal);

    /// Extract the minterms from a spot formula in DNF.
    /// The formula is true iff at least one minterm is satisfied.
    /// A minterm is a list of possibly negated atomic propositions.
    /// It is satisfied iff all its atomic propositions hold.
    /// @param formula A formula in disjunctive normal form
    /// @returns The minterms of the formula
    /// @throws std::invalid_argument If the formula is not in DNF
    std::vector<std::vector<std::pair<std::string, bool>>> extract_minterms_from_dnf(const spot::formula &formula);
}
