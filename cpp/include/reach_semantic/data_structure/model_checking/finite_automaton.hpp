#pragma once

#include <optional>
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

        /// Create a configured translator for converting LTLf formulas to automata.
        static spot::translator _get_translator();

    public:
        /// Construct the finite automaton accepting those words that make the given LTLf formula true
        /// @param ltlf_formulas The LTLf formulas to translate.
        /// @param mode How to combine the formulas into a single automaton:
        ///            0 = decide automatically
        ///            1 = translate individually and then compute the product automaton (faster)
        ///            2 = form conjunction of the formulas and translate the result (probably smaller automaton)
        explicit FiniteAutomaton(const std::vector<std::string> &ltlf_formulas, int mode = 0);

        /// The number of the initial state.
        unsigned int initial_state();

        /// Find all transitions outgoing from the given states.
        /// Tries to minimize the minterms by combining the conditions of the outgoing edges leading to the same destination.
        std::vector<std::pair<Minterm, unsigned int>>
        transitions_from(const std::set<unsigned int> &states);

        /// Check whether the given state is an accepting state.
        /// @returns True if and only if the state is accepting.
        bool is_accepting_state(unsigned int state);
    };
}
