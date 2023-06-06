#include <spot/misc/optionmap.hh>
#include <spot/twa/formula2bdd.hh>
#include <utility>
#include "reach_semantic/data_structure/model_checking/finite_automaton.hpp"
#include "reach_semantic/utility/spot.hpp"

using namespace semantic_reach;

FiniteAutomaton::FiniteAutomaton(const std::string &ltlf_formula) {
    spot::parsed_formula pf = spot::parse_infix_psl(ltlf_formula);
    if (pf.format_errors(std::cerr)) {
        throw std::runtime_error("Error while parsing LTLf formula.");
    }
    spot::option_map options = spot::option_map();
    // disable simulation based reductions to speed up translation
    // see https://spot.lre.epita.fr/man/spot-x.7.html
    options.set("simul", 0);
    spot::translator trans{&options};
    trans.set_type(spot::postprocessor::Buchi);
    trans.set_pref(spot::postprocessor::SBAcc | spot::postprocessor::Small);
    spot::twa_graph_ptr buechi_automaton = trans.run(spot::from_ltlf(pf.f));
    _spot_automaton = spot::to_finite(buechi_automaton);
    _bdict = _spot_automaton->get_dict();
}

unsigned int FiniteAutomaton::initial_state() {
    return _spot_automaton->get_init_state_number();
}

std::vector<std::pair<unsigned int, std::vector<Minterm>>> FiniteAutomaton::transitions_from(unsigned int state) {
    std::vector<std::pair<unsigned int, std::vector<Minterm>>> result{};
    for (auto &edge: _spot_automaton->out(state)) {
        result.emplace_back(edge.dst, _edge_condition_to_minterms(edge.cond));
    }
    return result;
}

std::vector<std::pair<unsigned int, std::vector<Minterm>>>
FiniteAutomaton::combined_transitions_from(const std::set<unsigned int> &states) {
    std::map<unsigned int, std::vector<bdd>> map_state_to_conditions{};
    for (const auto &state: states) {
        for (auto &edge: _spot_automaton->out(state)) {
            map_state_to_conditions[edge.dst].emplace_back(edge.cond);
        }
    }

    std::vector<std::pair<unsigned int, std::vector<Minterm>>> result{};
    for (const auto &[dst_state, conditions]: map_state_to_conditions) {
        bdd combined = bdd_false();
        for (const auto &condition: conditions) {
            combined = bdd_or(combined, condition);
        }
        result.emplace_back(dst_state, _edge_condition_to_minterms(std::move(combined)));
    }
    return result;
}

std::map<Minterm, std::set<unsigned int>>
FiniteAutomaton::non_deterministic_transitions_from(const std::set<unsigned int> &states) {
    std::map<Minterm, std::set<unsigned int>> transitions{};
    for (const auto &[next_state, minterms]: combined_transitions_from(states)) {
        for (const auto &minterm: minterms) {
            transitions[minterm].insert(next_state);
        }
    }
    return transitions;
}

bool FiniteAutomaton::is_accepting_state(unsigned int state) {
    return _spot_automaton->state_is_accepting(state);
}

std::vector<Minterm> FiniteAutomaton::_edge_condition_to_minterms(bdd cond) {
    // will be in DNF --> bbd_to_formula computes an irredundant sum of products
    // https://spot.lre.epita.fr/doxygen/namespacespot.html#aba9b9efe994006c29a6d77da94897df8
    auto formula_dnf = spot::bdd_to_formula(std::move(cond), _bdict);
    return util_spot::extract_minterms_from_dnf(formula_dnf);
}
