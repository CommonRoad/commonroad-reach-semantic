import commonroad_reach.utility.logger as util_logger
from commonroad_reach.data_structure.reach.reach_interface import ReachableSetInterface
from commonroad_reach.utility import visualization as util_visual

from commonroad_reach_semantic_addon.data_structure.reach.semantic_reach_set_py import PySemanticReachableSet
from commonroad_reach_semantic_addon.data_structure.semantic_configuration_builder import SemanticConfigurationBuilder
from commonroad_reach_semantic_addon.data_structure.semantic_model import SemanticModel
from commonroad_reach_semantic_addon.data_structure.spot_interface import SpotInterface
from commonroad_reach_semantic_addon.data_structure.traffic_rule_interface import TrafficRuleInterface


def main():
    # ==== specify scenario
    # name_scenario = "DEU_Test-1_1_T-1"
    # name_scenario = "ZAM_Over-1_1"
    # name_scenario = "ARG_Carcarana-1_1_T-1"
    # name_scenario = "USA_US101-6_1_T-1"
    name_scenario = "ZAM_Intersection-1_1_T-1"
    # name_scenario = "ZAM_Merge-1_1_T-1"

    # ==== build configuration
    config = SemanticConfigurationBuilder.build_configuration(name_scenario,
                                                              path_root="/home/lercher/tum/commonroad/commonroad-reach-semantic-addon")
    config.update()
    util_logger.initialize_logger(config)
    config.print_configuration_summary()

    # ==== initialize semantic model and traffic rules
    semantic_model = SemanticModel(config)
    rule_interface = TrafficRuleInterface(config)
    semantic_model.determine_traffic_priorities(rule_interface.dict_traffic_sign_to_priorities)
    rule_interface.concretize_traffic_rules(semantic_model)
    rule_interface.print_summary()

    # ==== compute reachable sets using reachability interface
    reach_interface = ReachableSetInterface(config)
    reach_interface._reach = PySemanticReachableSet(config, semantic_model, rule_interface)
    reach_interface.compute_reachable_sets()

    # ==== construct an interface to interact with Spot
    spot_interface = SpotInterface(reach_interface, rule_interface)
    spot_interface.translate_ltl_formulas()
    spot_interface.translate_reachability_graph()
    spot_interface.check()

    # ==== plot computation results
    # util_visual.plot_collision_checker(reach_interface)
    util_visual.plot_scenario_with_reachable_sets(reach_interface)


if __name__ == "__main__":
    main()
