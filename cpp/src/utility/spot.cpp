#include "reach_semantic/utility/spot.hpp"

std::vector<spot::formula> util_spot::conjuncts(const spot::formula &formula) {
    if (formula.is(spot::op::And)) {
        std::vector<spot::formula> result{};
        for (const auto &child : formula) {
            result.emplace_back(child);
        }
        return result;
    } else {
        return {formula};
    }
}

std::vector<spot::formula> util_spot::disjuncts(const spot::formula &formula) {
    if (formula.is(spot::op::Or)) {
        std::vector<spot::formula> result{};
        for (const auto &child : formula) {
            result.emplace_back(child);
        }
        return result;
    } else {
        return {formula};
    }
}

std::pair<std::string, bool> util_spot::extract_atomic_proposition(const spot::formula &literal) {
    if (literal.is(spot::op::Not) && literal[0].is(spot::op::ap)) {
        return {literal[0].ap_name(), true};
    } else if (literal.is(spot::op::ap)) {
        return {literal.ap_name(), false};
    } else {
        throw std::invalid_argument("Formula is not a literal or a negated literal.");
    }
}

std::vector<std::set<std::pair<std::string, bool>>>
util_spot::extract_minterms_from_dnf(const spot::formula &formula_dnf) {
    // We need to handle true and false separately, because they contain no literals we could extract
    if (formula_dnf.is(spot::op::tt)) {
        // true is trivially satisfied, so we return an empty minterm
        return {{}};
    } else if (formula_dnf.is(spot::op::ff)) {
        // false cannot be satisfied, so there are no minterms
        return {};
    } else {
        try {
            std::vector<std::set<std::pair<std::string, bool>>> minterms{};
            for (const auto &disjunct : disjuncts(formula_dnf)) {
                std::set<std::pair<std::string, bool>> minterm{};
                for (const auto &conjunct : conjuncts(disjunct)) {
                    minterm.emplace(extract_atomic_proposition(conjunct));
                }
                minterms.emplace_back(minterm);
            }
            return minterms;
        } catch (const std::invalid_argument &e) {
            throw std::invalid_argument("Formula is not in DNF.");
        }
    }
}
