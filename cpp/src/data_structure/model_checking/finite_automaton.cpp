#include <spot/misc/optionmap.hh>
#include <spot/twa/formula2bdd.hh>
#include <spot/twaalgos/product.hh>
#include <utility>
#include "reach_semantic/data_structure/model_checking/finite_automaton.hpp"
#include "reach_semantic/utility/spot.hpp"

using namespace semantic_reach;

FiniteAutomaton::FiniteAutomaton(const std::vector<std::string> &ltlf_formulas, int mode) {
    if (mode == 0) {
        // We always use the product automaton for now
        mode = 1;
    }

    std::vector<spot::formula> spot_formulas;
    spot_formulas.reserve(ltlf_formulas.size());
    std::transform(ltlf_formulas.begin(), ltlf_formulas.end(), std::back_inserter(spot_formulas), [](const std::string &f) {
        spot::parsed_formula pf = spot::parse_infix_psl(f);
        if (pf.format_errors(std::cerr)) {
            throw std::runtime_error("Error while parsing LTLf formula.");
        }
        return spot::from_ltlf(pf.f);
    });

    spot::option_map options{};
    // disable simulation based reductions to speed up translation
    // see https://spot.lre.epita.fr/man/spot-x.7.html
    options.set("simul", 0);
    spot::translator trans{&options};
    trans.set_type(spot::postprocessor::Buchi);
    trans.set_pref(spot::postprocessor::SBAcc | spot::postprocessor::Small);

    switch (mode) {
        case 1: {
            auto true_automaton = trans.run(spot::formula::tt());
            auto product_automaton = std::accumulate(spot_formulas.begin(), spot_formulas.end(), true_automaton, [&trans](const spot::twa_graph_ptr &acc, const spot::formula &f) {
                return spot::product(acc, trans.run(f));
            });
            _spot_automaton = spot::to_finite(product_automaton);
            break;
        }
        case 2: {
            auto conjunction = spot::formula::And(std::move(spot_formulas));
            _spot_automaton = spot::to_finite(trans.run(conjunction));
            break;
        }
        default: throw std::runtime_error("Invalid mode for combining LTLf formulas.");
    }

    _bdict = _spot_automaton->get_dict();
}

unsigned int FiniteAutomaton::initial_state() {
    return _spot_automaton->get_init_state_number();
}

std::vector<std::pair<std::vector<Minterm>, unsigned int>> FiniteAutomaton::transitions_from(unsigned int state) {
    std::vector<std::pair<std::vector<Minterm>, unsigned int>> result{};
    for (auto &edge: _spot_automaton->out(state)) {
        result.emplace_back(_edge_condition_to_minterms(edge.cond), edge.dst);
    }
    return result;
}

std::vector<std::pair<Minterm, unsigned int>>
FiniteAutomaton::combined_transitions_from(const std::set<unsigned int> &states) {
    std::map<unsigned int, std::vector<bdd>> map_state_to_conditions{};
    for (const auto &state: states) {
        for (auto &edge: _spot_automaton->out(state)) {
            map_state_to_conditions[edge.dst].emplace_back(edge.cond);
        }
    }

    std::vector<std::pair<Minterm, unsigned int>> result{};
    for (const auto &[dst_state, conditions]: map_state_to_conditions) {
        bdd combined{bdd_false()};
        for (const auto &condition: conditions) {
            combined = bdd_or(combined, condition);
        }
        auto minterms_to_dst = _edge_condition_to_minterms(std::move(combined));
        for (auto &&minterm: minterms_to_dst) {
            result.emplace_back(std::move(minterm), dst_state);
        }
    }
    return result;
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
