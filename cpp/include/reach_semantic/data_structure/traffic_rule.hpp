#pragma once

#include "reach_semantic/utility/shared_include.hpp"
#include "reachset/data_structure/reach/reach_node.hpp"
#include "reach_semantic/data_structure/semantic_configuration.hpp"
#include "reach_semantic/data_structure/proposition_holder.hpp"

namespace semantic_reach {
/// Class to hold adopted traffic rules
class TrafficRuleInterface {
public:
    py::handle obj_rule_interface_py;

    SemanticConfigurationPtr config;

    std::map<int, std::set<std::string>> map_step_to_propositions_mandatory;
    std::map<int, std::set<std::string>> map_step_to_propositions_forbidden;

    explicit TrafficRuleInterface(py::handle const& obj_traffic_rule_py);

    // Examines whether the given propagated sets satisfy the TPL specifications.
    vector<reach::ReachNodePtr>
    examine_tpl_specifications(int const &step, vector<reach::ReachNodePtr> const &vec_nodes_reach,
                               const std::map<reach::ReachNodePtr, PropositionHolder>& reachable_set_to_propositions);
};

using TrafficRuleInterfacePtr = std::shared_ptr<TrafficRuleInterface>;
}