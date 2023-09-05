import commonroad_reach.utility.logger as util_logger
from commonroad_reach.data_structure.reach.reach_interface import ReachableSetInterface

import commonroad_reach_semantic.data_structure.rule.priorities as priorities
from commonroad_reach_semantic.data_structure.config.semantic_configuration_builder import SemanticConfigurationBuilder
from commonroad_reach_semantic.data_structure.driving_corridor_extractor import DrivingCorridorExtractor
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.model_checking.spot_interface import SpotInterface
from commonroad_reach_semantic.data_structure.reach.semantic_labeling_reach_set_py import PySemanticLabelingReachableSet
from commonroad_reach_semantic.data_structure.reach.semantic_labeling_reach_set_cpp import CppSemanticLabelingReachableSet
from commonroad_reach_semantic.data_structure.reach.semantic_splitting_otf_reach_set_py import PySemanticSplittingOTFReachableSet
from commonroad_reach_semantic.data_structure.reach.semantic_splitting_otf_reach_set_cpp import CppSemanticSplittingOTFReachableSet
from commonroad_reach_semantic.data_structure.rule.traffic_rule_interface import TrafficRuleInterface
from commonroad_reach_semantic.utility import visualization as util_visual


def main():
    # ==== specify scenario
    # name_scenario = "DEU_Test-1_1_T-1"
    # name_scenario = "ZAM_Over-1_1"
    # name_scenario = "ARG_Carcarana-1_1_T-1"
    # name_scenario = "USA_US101-6_1_T-1"
    # name_scenario = "USA_US101-15_2_T-1"
    # name_scenario = "ZAM_Intersection-1_1_T-1"
    name_scenario = "ZAM_Intersection-1_2_T-1"
    # name_scenario = "ZAM_Merge-1_1_T-1"
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

    otf = True
    # otf = False

    # ==== compute reachable sets using reachability interface
    reach_interface = ReachableSetInterface(config)
    if not otf:
        reach_interface._reach = PySemanticLabelingReachableSet(config, semantic_model, rule_interface)
        # reach_interface._reach = CppSemanticLabelingReachableSet(config, semantic_model, rule_interface)
    else:
        reach_interface._reach = PySemanticSplittingOTFReachableSet(config, semantic_model, rule_interface)
        # reach_interface._reach = CppSemanticSplittingOTFReachableSet(config, semantic_model, rule_interface)
    reach_interface.compute_reachable_sets()

    # ==== construct an interface to interact with Spot
    if not otf:
        spot_interface = SpotInterface(reach_interface, rule_interface)
        spot_interface.translate_ltl_formulas()
        spot_interface.translate_reachability_graph()
        spot_interface.check()

    # ==== instantiate a driving corridor extractor
    # dc_extractor = DrivingCorridorExtractor(spot_interface)
    # dc_extractor.extract_corridors(search=True)
    # corridor_optimal = dc_extractor.determine_optimal_corridor()

    # return

    # ==== plot computation results
    node_to_group = util_visual.groups_from_states(reach_interface._reach.reachable_set_to_label) if otf \
        else util_visual.groups_from_propositions(reach_interface._reach.labeler.reachable_set_to_propositions)
    util_visual.plot_reach_graph(reach_interface, node_to_group=node_to_group)
    util_visual.plot_scenario_with_regions(semantic_model, "CVLN")
    util_visual.plot_scenario_with_reachable_sets(reach_interface, save_gif=True)
    if not otf:
        util_visual.plot_scenario_with_kripke_nodes(spot_interface, plot_accepting=True, save_gif=True)
    # util_visual.plot_scenario_with_driving_corridor(spot_interface, corridor_optimal, save_gif=True)

    # ==== show interactive visualization
    util_visual.show_interactive_reach_graph(reach_interface, use_images=True, node_to_group=node_to_group)


if __name__ == "__main__":
    main()
