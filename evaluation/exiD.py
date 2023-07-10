import glob
import os
import shutil
import time
from typing import Iterator, Callable

from multiprocessing import Pool

import commonroad_reach.utility.logger as util_logger
from commonroad_reach.data_structure.reach.reach_interface import ReachableSetInterface

import commonroad_reach_semantic.data_structure.rule.priorities as priorities
from commonroad_reach_semantic.data_structure.config.semantic_configuration import SemanticConfiguration
from commonroad_reach_semantic.data_structure.config.semantic_configuration_builder import SemanticConfigurationBuilder
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.model_checking.spot_interface import SpotInterface
from commonroad_reach_semantic.data_structure.reach.semantic_labeling_reach_set_py import PySemanticLabelingReachableSet
from commonroad_reach_semantic.data_structure.reach.semantic_labeling_reach_set_cpp import CppSemanticLabelingReachableSet
from commonroad_reach_semantic.data_structure.reach.semantic_otf_reach_set_py import PySemanticOTFReachableSet
from commonroad_reach_semantic.data_structure.reach.semantic_otf_reach_set_cpp import CppSemanticOTFReachableSet
from commonroad_reach_semantic.data_structure.reach.semantic_reach_interface import SemanticReachableSetInterface
from commonroad_reach_semantic.data_structure.reach.semantic_splitting_otf_reach_set_py import PySemanticSplittingOTFReachableSet
from commonroad_reach_semantic.data_structure.reach.semantic_splitting_otf_reach_set_cpp import CppSemanticSplittingOTFReachableSet
from commonroad_reach_semantic.data_structure.rule.traffic_rule_interface import TrafficRuleInterface
from commonroad_reach_semantic.utility import visualization as util_visual


def main():
    num_processes = 16
    scenario_paths = glob.glob("/home/lercher/datasets/exiD-commonroad-only6-merge/scenarios/DEU_MerzenichRather-*.xml")

    # copy_scenarios_from_file_list(
    #     "evaluation/driving_right_scenarios.txt",
    #     "/home/lercher/datasets/exiD-commonroad-only6/scenarios",
    #     "/home/lercher/datasets/exiD-commonroad-only6-merge/scenarios",
    # )

    # run_parallel(
    #     lambda n: run_with_except(n, filter_scenario),
    #     (os.path.splitext(os.path.basename(path))[0] for path in scenario_paths),
    #     num_processes=num_processes,
    # )

    # name = "DEU_MerzenichRather-2_882250_T-2399"  # vehicle not in local road network
    # name = "DEU_MerzenichRather-2_887500_T-7649"  # successful scenario
    # name = "DEU_MerzenichRather-2_887050_T-7199"  # conversion initial state to CLCS failed
    # name = "DEU_MerzenichRather-2_850_T-149"  # scenario, where rule should actually cut some states, but entering vehicle is very far away
    # name = "DEU_MerzenichRather-2_9223250_T-23399"  # scenario, where rule should actually cut some states
    # name = "DEU_MerzenichRather-2_7915150_T-15299"  # already in right lane
    name = "DEU_MerzenichRather-2_8814400_T-14549"  # very good scenario with two entering vehicles, but only one reaches main carriageway
    run_scenario(name, draw=False, otf=True)


def scenarios_from_file(path: str) -> Iterator[str]:
    with open(path) as f:
        for line in f:
            yield line.strip()


def run_scenario(name: str, draw: bool = False, otf: bool = True, path_root: str = "/home/lercher/datasets/exiD-commonroad-only6-merge") -> None:
    # ==== build configuration
    config = SemanticConfigurationBuilder.build_configuration(name, path_root=path_root)

    config.update()
    util_logger.initialize_logger(config)
    config.print_configuration_summary()

    # ==== initialize semantic model and traffic rules
    semantic_model = SemanticModel(config)
    semantic_model.determine_traffic_priorities(priorities.dict_traffic_sign_to_priorities)
    rule_interface = TrafficRuleInterface(config, semantic_model)
    rule_interface.print_summary()

    # ==== compute reachable sets using reachability interface
    time_start = time.perf_counter()
    reach_interface = SemanticReachableSetInterface(config, semantic_model, rule_interface)
    if not otf:
        # reach_interface.set_reach(PySemanticLabelingReachableSet(config, semantic_model, rule_interface))
        reach_interface.set_reach(CppSemanticLabelingReachableSet(config, semantic_model, rule_interface))
    else:
        # reach_interface.set_reach(PySemanticOTFReachableSet(config, semantic_model, rule_interface))
        # reach_interface.set_reach(CppSemanticOTFReachableSet(config, semantic_model, rule_interface))
        reach_interface.set_reach(PySemanticSplittingOTFReachableSet(config, semantic_model, rule_interface))
        # reach_interface.set_reach(CppSemanticSplittingOTFReachableSet(config, semantic_model, rule_interface))
    reach_interface.compute_reachable_sets()
    overall_time = time.perf_counter() - time_start
    print(f"Overall time: {overall_time:.3f}s")

    benchmark_result = reach_interface._reach.benchmark_result
    print(benchmark_result)

    # ==== construct an interface to interact with Spot
    if not otf:
        time_start = time.perf_counter()
        spot_interface = SpotInterface(reach_interface, rule_interface)
        spot_interface.translate_ltl_formulas()
        spot_interface.translate_reachability_graph()
        spot_interface.check()
        model_checking_time = time.perf_counter() - time_start
        print(f"Model checking time: {model_checking_time:.3f}s")

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

    # ==== show interactive visualization
    util_visual.show_interactive_reach_graph(reach_interface, use_images=True, node_to_group=node_to_group)


def filter_scenario(name: str, path_root: str = "/home/lercher/datasets/exiD-commonroad-only6-merge") -> None:
    # ==== build configuration
    config = SemanticConfigurationBuilder.build_configuration(name, path_root=path_root)
    config.update()
    util_logger.initialize_logger(config)

    # ==== initialize semantic model and traffic rules
    semantic_model = SemanticModel(config)
    semantic_model.determine_traffic_priorities(priorities.dict_traffic_sign_to_priorities)
    if not is_good_initial_state(semantic_model, config):
        return

    rule_interface = TrafficRuleInterface(config, semantic_model)
    if not len(rule_interface.list_specifications_ltl) > 1:
        return

    print(name)


def run_parallel(func: Callable[[str], None], names: Iterator[str], num_processes: int = 16) -> None:
    with Pool(num_processes) as p:
        p.map(func, names)


def run_with_except(name: str, func: Callable[[str], None]) -> None:
    try:
        func(name)
    except Exception as e:
        print("failed:", name)
        print(e)


def is_good_initial_state(semantic_model: SemanticModel, config: SemanticConfiguration) -> bool:
    [list_ids_lanelets] = semantic_model.lanelet_model.road_network.lanelet_network.find_lanelet_by_position(
        [config.planning_problem.initial_state.position])
    good_lanelets = [10, 11, 12, 13, 14, 15, 17, 18, 19, 31, 33, 35, 32, 34, 36]
    for id_lanelet in list_ids_lanelets:
        if id_lanelet in good_lanelets:
            return True
    return False


def copy_scenarios_from_file_list(list_path: str, src_dir: str, dst_dir: str) -> None:
    for name in scenarios_from_file(list_path):
        src = os.path.join(src_dir, name + ".xml")
        dst = os.path.join(dst_dir, name + ".xml")
        shutil.copy(src, dst)


if __name__ == "__main__":
    main()
