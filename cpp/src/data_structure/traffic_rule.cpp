#include "reach_semantic/data_structure/traffic_rule.hpp"
#include "reachset/utility/shared_using.hpp"

using namespace semantic_reach;

TrafficRuleInterface::TrafficRuleInterface(py::handle const &obj_traffic_rule_py) {
    this->obj_rule_interface_py = obj_traffic_rule_py;

    auto dict_step_to_propositions_mandatory =
            obj_traffic_rule_py.attr("tpl_checker").attr("dict_step_to_propositions_mandatory");
    auto dict_step_to_propositions_forbidden =
            obj_traffic_rule_py.attr("tpl_checker").attr("dict_step_to_propositions_forbidden");

    for (auto const &step: dict_step_to_propositions_mandatory) {
        auto set_propositions_mandatory = dict_step_to_propositions_mandatory[step];
        auto set_propositions_forbidden = dict_step_to_propositions_forbidden[step];

        map_step_to_propositions_mandatory[step.cast<int>()] = set_propositions_mandatory.cast<std::set<string>>();
        map_step_to_propositions_forbidden[step.cast<int>()] = set_propositions_forbidden.cast<std::set<string>>();
    }

    vec_specifications_ltl = obj_traffic_rule_py.attr("list_specifications_ltl").cast<std::vector<std::string>>();
}

vector<reach::ReachNodePtr>
TrafficRuleInterface::examine_tpl_specifications(int const &step, vector<reach::ReachNodePtr> const &vec_nodes_reach,
                                                 const std::map<reach::ReachNodePtr, PropositionHolder> &reachable_set_to_propositions) {
    vector<reach::ReachNodePtr> vec_nodes_keep{};

    auto const &set_propositions_mandatory = map_step_to_propositions_mandatory[step];
    auto const &set_propositions_forbidden = map_step_to_propositions_forbidden[step];

    for (auto const &node: vec_nodes_reach) {
        auto const &set_propositions_node = reachable_set_to_propositions.at(node).set_propositions;
        bool includes_mandatory = std::includes(set_propositions_node.begin(), set_propositions_node.end(),
                                                set_propositions_mandatory.begin(),
                                                set_propositions_mandatory.end());

        set<string> intersection{};
        set_intersection(set_propositions_node.begin(), set_propositions_node.end(),
                         set_propositions_forbidden.begin(), set_propositions_forbidden.end(),
                         std::inserter(intersection, intersection.begin()));

        bool excludes_forbidden = intersection.empty();

        if (includes_mandatory and excludes_forbidden) {
            vec_nodes_keep.emplace_back(node);
        }
    }

    return vec_nodes_keep;
}
