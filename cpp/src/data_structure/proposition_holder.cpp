#include "reachset/data_structure/proposition_holder.hpp"
#include "reachset/utility/shared_using.hpp"

using namespace reach;

PropositionHolder::PropositionHolder(std::set<std::string> const& set_propositions, PropositionGroup const& group) {
    add_propositions(set_propositions, group);
}

MultiStepPropositionHolder::MultiStepPropositionHolder(int const& step_end) {
    for (auto step = 0; step < step_end; step++) {
        map_step_to_set_propositions_time_variant[step] = set<string>();
    }
}

MultiStepPropositionHolder::MultiStepPropositionHolder(py::handle const& obj_ph_py) {
    //
    set_propositions_time_invariant = obj_ph_py.attr("set_propositions_time_invariant").cast<set<string>>();
    //
    map_step_to_set_propositions_time_variant =
            obj_ph_py.attr("dict_step_to_set_propositions_time_variant").cast<map<int, set<string>>>();
    //
    for (auto const& [group_py, set_propositions_time_invariant_py]:
            obj_ph_py.attr("dict_group_to_set_propositions_time_invariant").cast<py::dict>()) {

        auto group = get_proposition_group(group_py.attr("name").cast<string>());
        map_group_to_set_propositions_time_invariant[group] = set_propositions_time_invariant_py.cast<set<string>>();
    }
    //
    for (auto const& [group_py, dict_step_to_set_propositions_time_variant_py]:
            obj_ph_py.attr("dict_group_to_step_to_set_propositions_time_variant").cast<py::dict>()) {

        auto group = get_proposition_group(group_py.attr("name").cast<string>());
        map_group_to_step_to_set_propositions_time_variant[group] = map<int, set<string>>{};

        for (auto const& [step_py, set_propositions_time_variant_py]:
                dict_step_to_set_propositions_time_variant_py.cast<py::dict>()) {
            map_group_to_step_to_set_propositions_time_variant[group][step_py.cast<int>()] =
                    set_propositions_time_variant_py.cast<set<string>>();
        }
    }
}