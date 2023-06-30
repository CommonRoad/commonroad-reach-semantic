#pragma once

#include <spot/tl/parse.hh>
#include <spot/tl/ltlf.hh>
#include <spot/twaalgos/translate.hh>
#include <spot/twaalgos/hoa.hh>
#include <spot/twaalgos/remprop.hh>

namespace semantic_reach {
    /// Represents a finite automaton on words over the powerset of propositions.
    using Literal = std::pair<std::string, bool>;
    using Minterm = std::set<Literal>;

    class FiniteAutomaton {
    private:
        spot::twa_graph_ptr _spot_automaton;
        spot::bdd_dict_ptr _bdict;

        /// Convert a condition on an automaton edge given as a BDD into a list of minterms.
        /// The condition is true iff at least one minterm is satisfied.
        /// A minterm is a list of possible negated atomic propositions.
        /// It is satisfied iff all its atomic propositions hold.
        std::vector<Minterm> _edge_condition_to_minterms(bdd cond);

    public:
        /// Construct the finite automaton accepting those words that make the given LTLf formula true
        /// @param ltlf_formula The LTLf formula to translate.
        FiniteAutomaton(const std::string &ltlf_formula);

        /// The number of the initial state.
        unsigned int initial_state();

        /// Find all transitions outgoing from the given state.
        std::vector<std::pair<std::vector<Minterm>, unsigned int>> transitions_from(unsigned int state);

        /// Find all transitions outgoing from the given states.
        /// Tries to minimize the minterms by combining the conditions of the outgoing edges leading to the same destination.
        std::vector<std::pair<Minterm, unsigned int>>
        combined_transitions_from(const std::set<unsigned int> &states);

        /// Check whether the given state is an accepting state.
        /// @returns True if and only if the state is accepting.
        bool is_accepting_state(unsigned int state);
    };
}
