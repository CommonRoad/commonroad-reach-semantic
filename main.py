from collections import defaultdict

import commonroad_reach.utility.logger as util_logger

import commonroad_reach_semantic.data_structure.rule.priorities as priorities
from commonroad_reach_semantic.data_structure.config.semantic_configuration_builder import SemanticConfigurationBuilder
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.model_checking.spot_interface import SpotInterface
from commonroad_reach_semantic.data_structure.reach.semantic_reach_interface import SemanticReachableSetInterface
from commonroad_reach_semantic.data_structure.rule.traffic_rule_interface import TrafficRuleInterface
from commonroad_reach_semantic.utility import visualization as util_visual


def main():
    # ==== specify scenario
    # name_scenario = "ZAM_Intersection-1_1_T-1"
    # name_scenario = "ZAM_Intersection-1_2_T-1"
    # name_scenario = "ZAM_Merge-1_1_T-1"
    name_scenario = "ZAM_Yield-1_1_T-1"
    # name_scenario = "ESP_Monzon-2_2_T-1"

    # ==== build configuration
    path_root = "/home/lercher/tum/commonroad/commonroad-reach-semantic-addon"
    config = SemanticConfigurationBuilder(path_root=path_root).build_configuration(name_scenario)
    config.update()
    util_logger.initialize_logger(config)
    config.print_configuration_summary()

    # ==== initialize semantic model and traffic rules
    semantic_model = SemanticModel(config)
    semantic_model.determine_traffic_priorities(priorities.dict_traffic_sign_to_priorities)
    rule_interface = TrafficRuleInterface(config, semantic_model)
    rule_interface.print_summary()

    # ==== compute reachable sets using reachability interface
    reach_interface = SemanticReachableSetInterface(config, semantic_model, rule_interface)
    reach_interface.compute_reachable_sets()

    # ==== plot computation results
    if config.reachable_set.mode_computation in [5, 6]:
        node_to_group = util_visual.groups_from_propositions(
            reach_interface._reach.labeler.reachable_set_to_propositions)
    elif config.reachable_set.mode_computation in [7, 8]:
        node_to_group = util_visual.groups_from_states(reach_interface._reach.reachable_set_to_label)
    else:
        # no semantic information, so put all nodes in the same group
        node_to_group = defaultdict(lambda: 0)

    util_visual.plot_reach_graph(reach_interface, node_to_group=node_to_group)
    util_visual.plot_scenario_with_regions(semantic_model, "CVLN")
    util_visual.plot_scenario_with_reachable_sets(reach_interface, save_gif=True)

    # ==== show interactive visualization (can take a long time to plot if there are many nodes)
    # util_visual.show_interactive_reach_graph(reach_interface, use_images=True, node_to_group=node_to_group)

    if config.reachable_set.mode_computation in [5, 6]:
        # only necessary for labeling reachable set
        spot_interface = SpotInterface(reach_interface, rule_interface)
        spot_interface.translate_ltl_formulas()
        spot_interface.translate_reachability_graph()
        spot_interface.check()
        util_visual.plot_scenario_with_kripke_nodes(spot_interface, plot_accepting=True, save_gif=True)


if __name__ == "__main__":
    main()
