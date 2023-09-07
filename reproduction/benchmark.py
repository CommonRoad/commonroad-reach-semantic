import os
import time
from datetime import datetime
from typing import Tuple, List

import commonroad_reach.utility.logger as util_logger
import yaml
from alive_progress import alive_bar

import commonroad_reach_semantic.data_structure.rule.priorities as priorities
from commonroad_reach_semantic.benchmark.benchmark_result import ReachBenchmarkResults
from commonroad_reach_semantic.data_structure.config.semantic_configuration import SemanticConfiguration
from commonroad_reach_semantic.data_structure.config.semantic_configuration_builder import SemanticConfigurationBuilder
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.model_checking.spot_interface import SpotInterface
from commonroad_reach_semantic.data_structure.reach.semantic_labeling_reach_set_cpp import \
    CppSemanticLabelingReachableSet
from commonroad_reach_semantic.data_structure.reach.semantic_labeling_reach_set_py import PySemanticLabelingReachableSet
from commonroad_reach_semantic.data_structure.reach.semantic_reach_interface import SemanticReachableSetInterface
from commonroad_reach_semantic.data_structure.reach.semantic_splitting_otf_reach_set_cpp import \
    CppSemanticSplittingOTFReachableSet
from commonroad_reach_semantic.data_structure.reach.semantic_splitting_otf_reach_set_py import \
    PySemanticSplittingOTFReachableSet
from commonroad_reach_semantic.data_structure.rule.traffic_rule_interface import TrafficRuleInterface
from commonroad_reach_semantic.utility import visualization as util_visual


def benchmark_with_progress(scenario_names: List[str], mode: int, repetitions: int = 5, cpp: bool = True,
                            path_root: str = "/home/lercher/datasets/exiD-commonroad-only6-merge",
                            output_dir: str = "benchmark"):
    with alive_bar(len(scenario_names)) as bar:
        for name in scenario_names:
            print(f"Running benchmark for {name}")
            bar.text(name)
            benchmark_scenario(name, mode, repetitions=repetitions, cpp=cpp, path_root=path_root, output_dir=output_dir)
            bar()


def benchmark_scenario(name: str, mode: int, repetitions: int = 5, cpp: bool = True,
                       path_root: str = "/home/lercher/datasets/exiD-commonroad-only6-merge",
                       output_dir: str = "benchmark"):
    # modes: 0 = both, 1 = OTF, 2 = Labeling
    config = SemanticConfigurationBuilder(path_root=path_root).build_configuration(name)
    config.update()
    util_logger.initialize_logger(config)

    semantic_model = SemanticModel(config)
    semantic_model.determine_traffic_priorities(priorities.dict_traffic_sign_to_priorities)

    rule_interface = TrafficRuleInterface(config, semantic_model)

    otf_results = []
    labeling_results = []
    for _ in range(repetitions):
        if mode == 0 or mode == 1:
            otf_res = run_prepared_otf_scenario(config, semantic_model, rule_interface, cpp)
            otf_results.append(otf_res)
        if mode == 0 or mode == 2:
            labeling_res = run_prepared_labeling_scenario(config, semantic_model, rule_interface, cpp)
            labeling_results.append(labeling_res)

    output_path = os.path.join(path_root, output_dir)
    os.makedirs(output_path, exist_ok=True)
    if mode == 0 or mode == 1:
        write_otf_benchmark_results_to_file(name, otf_results, os.path.join(path_root, output_dir))
    if mode == 0 or mode == 2:
        write_labeling_benchmark_results_to_file(name, labeling_results, os.path.join(path_root, output_dir))


def run_prepared_labeling_scenario(config: SemanticConfiguration, semantic_model: SemanticModel,
                                   rule_interface: TrafficRuleInterface, cpp: bool = True) -> Tuple[
    ReachBenchmarkResults, float, float]:
    time_start = time.perf_counter()
    reach_interface = SemanticReachableSetInterface(config, semantic_model, rule_interface)
    if cpp:
        reach = CppSemanticLabelingReachableSet(config, semantic_model, rule_interface)
    else:
        reach = PySemanticLabelingReachableSet(config, semantic_model, rule_interface)
    reach_interface.set_reach(reach)
    reach_interface.compute_reachable_sets()
    overall_time = time.perf_counter() - time_start
    benchmark_result = reach_interface._reach.benchmark_result

    # ==== construct an interface to interact with Spot
    time_start = time.perf_counter()
    spot_interface = SpotInterface(reach_interface, rule_interface)
    spot_interface.translate_ltl_formulas()
    spot_interface.translate_reachability_graph()
    spot_interface.check()
    model_checking_time = time.perf_counter() - time_start

    return benchmark_result, overall_time, model_checking_time


def run_prepared_otf_scenario(config: SemanticConfiguration, semantic_model: SemanticModel,
                              rule_interface: TrafficRuleInterface, cpp: bool = True) -> Tuple[
    ReachBenchmarkResults, float]:
    time_start = time.perf_counter()
    reach_interface = SemanticReachableSetInterface(config, semantic_model, rule_interface)
    if cpp:
        reach = CppSemanticSplittingOTFReachableSet(config, semantic_model, rule_interface)
    else:
        reach = PySemanticSplittingOTFReachableSet(config, semantic_model, rule_interface)
    reach_interface.set_reach(reach)
    reach_interface.compute_reachable_sets()
    overall_time = time.perf_counter() - time_start
    benchmark_result = reach_interface._reach.benchmark_result

    return benchmark_result, overall_time


def write_otf_benchmark_results_to_file(scenario_name: str, results: List[Tuple[ReachBenchmarkResults, float]],
                                        file_path: str):
    yaml_dict = {
        "scenario_name": scenario_name,
    }
    result_dicts = []
    for benchmark_result, overall_time in results:
        result_dict = benchmark_result.to_dict()
        result_dict["overall_time"] = overall_time
        result_dicts.append(result_dict)
    yaml_dict["results"] = result_dicts
    now = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    with open(os.path.join(file_path, f"O_{scenario_name}_{now}.yaml"), "w") as f:
        yaml.dump(yaml_dict, f)


def write_labeling_benchmark_results_to_file(scenario_name: str,
                                             results: List[Tuple[ReachBenchmarkResults, float, float]], file_path: str):
    yaml_dict = {
        "scenario_name": scenario_name,
    }
    result_dicts = []
    for benchmark_result, overall_time, model_checking_time in results:
        result_dict = benchmark_result.to_dict()
        result_dict["overall_time"] = overall_time
        result_dict["model_checking_time"] = model_checking_time
        result_dicts.append(result_dict)
    yaml_dict["results"] = result_dicts
    now = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    with open(os.path.join(file_path, f"L_{scenario_name}_{now}.yaml"), "w") as f:
        yaml.dump(yaml_dict, f)

def run_scenario(name: str, draw: bool = False, interactive_viz: bool = False, otf: bool = True, cpp: bool = False, path_root: str = "/home/lercher/datasets/exiD-commonroad-only6-merge") -> None:
    # ==== build configuration
    config = SemanticConfigurationBuilder(path_root=path_root).build_configuration(name)

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
    if not otf:
        if cpp:
            reach_interface.set_reach(CppSemanticLabelingReachableSet(config, semantic_model, rule_interface))
        else:
            reach_interface.set_reach(PySemanticLabelingReachableSet(config, semantic_model, rule_interface))
    else:
        if cpp:
            reach_interface.set_reach(CppSemanticSplittingOTFReachableSet(config, semantic_model, rule_interface))
        else:
            reach_interface.set_reach(PySemanticSplittingOTFReachableSet(config, semantic_model, rule_interface))
    reach_interface.compute_reachable_sets()

    if not otf:
        # ==== construct an interface to interact with Spot
        spot_interface = SpotInterface(reach_interface, rule_interface)
        spot_interface.translate_ltl_formulas()
        spot_interface.translate_reachability_graph()
        spot_interface.check()

    node_to_group = util_visual.groups_from_states(reach_interface._reach.reachable_set_to_label) if otf \
        else util_visual.groups_from_propositions(reach_interface._reach.labeler.reachable_set_to_propositions)

    if draw:
        # ==== plot computation results
        suffix_lang = "cpp" if cpp else "py"
        suffix_mode = "otf" if otf else "labeling"
        output_path = os.path.join(os.path.dirname(os.path.normpath(config.general.path_output)),
                                   f"{name}.{suffix_lang}.{suffix_mode}")

        util_visual.plot_reach_graph(reach_interface, node_to_group=node_to_group, path_output=output_path)
        util_visual.plot_scenario_with_regions(semantic_model, "CVLN", path_output=output_path)
        util_visual.plot_scenario_with_reachable_sets(semantic_model, reach_interface, save_gif=False, path_output=output_path)

        if not otf:
            util_visual.plot_scenario_with_kripke_nodes(semantic_model, spot_interface, plot_accepting=True, save_gif=False, path_output=output_path)

    if interactive_viz:
        # ==== show interactive visualization
        util_visual.show_interactive_reach_graph(reach_interface, use_images=True, node_to_group=node_to_group)
