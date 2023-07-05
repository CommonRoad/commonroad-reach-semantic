import glob
import os
import shutil
from typing import Iterator

from alive_progress import alive_bar
from multiprocessing import Pool

import commonroad_reach.utility.logger as util_logger
from commonroad_reach.data_structure.reach.reach_interface import ReachableSetInterface
from commonroad_reach.utility import configuration as util_configuration

import commonroad_reach_semantic.data_structure.rule.priorities as priorities
from commonroad_reach_semantic.data_structure.config.semantic_configuration import SemanticConfiguration
from commonroad_reach_semantic.data_structure.config.semantic_configuration_builder import SemanticConfigurationBuilder
from commonroad_reach_semantic.data_structure.driving_corridor_extractor import DrivingCorridorExtractor
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.model_checking.spot_interface import SpotInterface
from commonroad_reach_semantic.data_structure.reach.semantic_labeling_reach_set_py import PySemanticLabelingReachableSet
from commonroad_reach_semantic.data_structure.reach.semantic_labeling_reach_set_cpp import CppSemanticLabelingReachableSet
from commonroad_reach_semantic.data_structure.reach.semantic_otf_reach_set_py import PySemanticOTFReachableSet
from commonroad_reach_semantic.data_structure.reach.semantic_otf_reach_set_cpp import CppSemanticOTFReachableSet
from commonroad_reach_semantic.data_structure.reach.semantic_splitting_otf_reach_set_py import PySemanticSplittingOTFReachableSet
from commonroad_reach_semantic.data_structure.reach.semantic_splitting_otf_reach_set_cpp import CppSemanticSplittingOTFReachableSet
from commonroad_reach_semantic.data_structure.rule.traffic_rule_interface import TrafficRuleInterface
from commonroad_reach_semantic.utility import visualization as util_visual


def main():
    num_processes = 15
    scenario_paths = glob.glob("/home/lercher/datasets/exiD-commonroad-only6-merge/scenarios/DEU_MerzenichRather-*.xml")

    # scenario_names = list(scenarios_from_file("evaluation/driving_right_scenarios.txt"))
    # base = "/home/lercher/datasets"
    # for name in scenario_names:
    #     src = os.path.join(base, "exiD-commonroad-only6", "scenarios", name + ".xml")
    #     dst = os.path.join(base, "exiD-commonroad-only6-merge", "scenarios", name + ".xml")
    #     shutil.copy(src, dst)

    # with Pool(num_processes) as p:
    #     p.map(run_with_except, (os.path.splitext(os.path.basename(path))[0] for path in scenario_paths))
    # with alive_bar(len(scenario_paths), force_tty=True) as bar:
    #     for path in scenario_paths:
    #         name = os.path.splitext(os.path.basename(path))[0]
    #         bar.text(name)
    #         run_with_except(name)
    #         bar()
    # name = "DEU_MerzenichRather-2_882250_T-2399"  # vehicle not in local road network
    # name = "DEU_MerzenichRather-2_887500_T-7649"  # successful scenario
    # name = "DEU_MerzenichRather-2_887050_T-7199"  # conversion initial state to CLCS failed
    # name = "DEU_MerzenichRather-2_850_T-149"  # scenario, where rule should actually cut some states, but entering vehicle is very far away
    # name = "DEU_MerzenichRather-2_9223250_T-23399"  # scenario, where rule should actually cut some states
    # name = "DEU_MerzenichRather-2_7915150_T-15299"  # already in right lane
    name = "DEU_MerzenichRather-2_8814400_T-14549"  # very good scenario with two entering vehicles, but only one reaches main carriageway
    run_scenario(name, draw=True, otf=True)


def scenarios_from_file(path: str) -> Iterator[str]:
    with open(path) as f:
        for line in f:
            yield line.strip()


def run_with_except(name: str):
    try:
        run_scenario(name)
    except ValueError as e:
        if "Coordinate outside of projection domain" in e.args[0]:
            print("failed CLCS:", name)
        else:
            print("failed:", name)
            print(e)
    except Exception as e:
        print("failed:", name)
        print(e)


def run_scenario(name: str, draw: bool = False, otf: bool = True):
    # ==== build configuration
    config = SemanticConfigurationBuilder.build_configuration(name,
                                                              path_root="/home/lercher/datasets/exiD-commonroad-only6-merge")

    config.update()
    util_logger.initialize_logger(config)
    config.print_configuration_summary()

    # ==== initialize semantic model and traffic rules
    semantic_model = SemanticModel(config)
    semantic_model.determine_traffic_priorities(priorities.dict_traffic_sign_to_priorities)

    # if is_good_initial_state(semantic_model, config):
    #     print(name)
    # return

    rule_interface = TrafficRuleInterface(config, semantic_model)
    rule_interface.print_summary()

    # if len(rule_interface.list_specifications_ltl) > 1:
    #     print(name)
    # return


    # ==== compute reachable sets using reachability interface
    reach_interface = ReachableSetInterface(config)
    if not otf:
        # reach_interface._reach = PySemanticLabelingReachableSet(config, semantic_model, rule_interface)
        reach_interface._reach = CppSemanticLabelingReachableSet(config, semantic_model, rule_interface)
    else:
        # reach_interface._reach = PySemanticOTFReachableSet(config, semantic_model, rule_interface)
        # reach_interface._reach = CppSemanticOTFReachableSet(config, semantic_model, rule_interface)
        # reach_interface._reach = PySemanticSplittingOTFReachableSet(config, semantic_model, rule_interface)
        reach_interface._reach = CppSemanticSplittingOTFReachableSet(config, semantic_model, rule_interface)
    reach_interface.compute_reachable_sets()

    # ==== construct an interface to interact with Spot
    if not otf:
        spot_interface = SpotInterface(reach_interface, rule_interface)
        spot_interface.translate_ltl_formulas()
        spot_interface.translate_reachability_graph()
        spot_interface.check()

    if not draw:
        return

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
    # util_visual.show_interactive_reach_graph(reach_interface, use_images=True, node_to_group=node_to_group)


def is_good_initial_state(semantic_model: SemanticModel, config: SemanticConfiguration) -> bool:
    [list_ids_lanelets] = semantic_model.lanelet_model.road_network.lanelet_network.find_lanelet_by_position(
        [config.planning_problem.initial_state.position])
    good_lanelets = [10, 11, 12, 13, 14, 15, 17, 18, 19, 31, 33, 35, 32, 34, 36]
    for id_lanelet in list_ids_lanelets:
        if id_lanelet in good_lanelets:
            return True
    return False


if __name__ == "__main__":
    main()
