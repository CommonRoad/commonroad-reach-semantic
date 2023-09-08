import glob
import os
from dataclasses import dataclass
from typing import Tuple, Optional, List

import matplotlib.pyplot as plt
import pandas as pd
import yaml

from commonroad_reach_semantic.benchmark.benchmark_result import ReachBenchmarkResults, ReachComputationTimes


@dataclass(kw_only=True, frozen=True)
class SummaryBenchmarkResult:
    scenario_name: str
    automaton_creation: float
    other_initialization: float
    pruning: float
    propagation: float
    splitting: float
    partitioning: float
    collision_check: float
    merge: float
    node_creation: float
    overall: float
    nodes_before_pruning: int
    nodes_after_pruning: int
    model_checking: Optional[float] = None


def otf_labeling_comparison(path_benchmark_dir: str) -> pd.DataFrame:
    computation_columns = ["propagation", "splitting", "partitioning", "collision_check", "merge", "node_creation", "pruning"]

    otf_scenario_paths = glob.glob(os.path.join(path_benchmark_dir, "O_*.yaml"))
    otf_results = pd.concat((read_otf_results(path) for path in otf_scenario_paths), ignore_index=True, sort=False)
    otf_results["computation"] = otf_results[computation_columns].sum(axis=1)
    otf_aggregated_per_scenario = otf_results.groupby("scenario_name").mean()

    labeling_scenario_paths = glob.glob(os.path.join(path_benchmark_dir, "L_*.yaml"))
    labeling_results = pd.concat((read_labeling_results(path) for path in labeling_scenario_paths), ignore_index=True, sort=False)
    labeling_results["computation"] = labeling_results[computation_columns].sum(axis=1)
    labeling_aggregated_per_scenario = labeling_results.groupby("scenario_name").mean()

    combined = pd.merge(
        otf_aggregated_per_scenario[["automaton_creation", "computation", "nodes_before_pruning", "nodes_after_pruning"]],
        labeling_aggregated_per_scenario[["model_checking", "computation", "nodes_before_pruning", "nodes_after_pruning"]],
        on="scenario_name", suffixes=("_otf", "_lab")
    )
    combined["pure_model_checking"] = combined["model_checking"] - combined["automaton_creation"]

    return combined


def boxplot_computation_times_otf(path_benchmark_dir: str, show_plot: bool = False) -> List[
        Tuple[str, float, Tuple[float, float], Tuple[float, float]]]:
    """Return values for boxplot of computation times for OTF scenarios.

    :param path_benchmark_dir: Path to directory containing the benchmark results.
    :param show_plot: Show the plot with matplotlib.
    :return: Tuple of column name, median, box (lower, upper), whisker (lower, upper).
    """
    scenario_paths = glob.glob(os.path.join(path_benchmark_dir, "O_*.yaml"))
    results = pd.concat((read_otf_results(path) for path in scenario_paths), ignore_index=True, sort=False)

    aggregated_per_scenario = results.groupby("scenario_name").mean()
    for_boxplot = aggregated_per_scenario[["automaton_creation", "pruning"]].copy()
    for_boxplot["propagation"] = aggregated_per_scenario[["propagation"]].sum(axis=1)
    for_boxplot["splitting"] = aggregated_per_scenario[["splitting", "collision_check"]].sum(axis=1)
    for_boxplot["repartitioning"] = aggregated_per_scenario[["partitioning", "merge", "node_creation"]].sum(axis=1)
    columns = ["automaton_creation", "pruning", "propagation", "splitting", "repartitioning"]
    for_boxplot["total"] = for_boxplot[columns].sum(axis=1, numeric_only=True)
    columns.append("total")

    return create_boxplot(for_boxplot, columns, show_plot)


def boxplot_computation_times_labeling(path_benchmark_dir: str, show_plot: bool = False) -> List[
        Tuple[str, float, Tuple[float, float], Tuple[float, float]]]:
    """Return values for boxplot of computation times for labeling scenarios.

    :param path_benchmark_dir: Path to directory containing the benchmark results.
    :param show_plot: Show the plot with matplotlib.
    :return: Tuple of column name, median, box (lower, upper), whisker (lower, upper).
    """
    scenario_paths = glob.glob(os.path.join(path_benchmark_dir, "L_*.yaml"))
    results = pd.concat((read_otf_results(path) for path in scenario_paths), ignore_index=True, sort=False)

    aggregated_per_scenario = results.groupby("scenario_name").mean()
    for_boxplot = aggregated_per_scenario[["pruning"]].copy()
    for_boxplot["propagation"] = aggregated_per_scenario[["propagation"]].sum(axis=1)

    for_boxplot["splitting"] = aggregated_per_scenario[["splitting"]].sum(axis=1)
    for_boxplot["collision_check"] = aggregated_per_scenario[["collision_check"]].sum(axis=1)
    for_boxplot["repartitioning"] = aggregated_per_scenario[["partitioning", "merge", "node_creation"]].sum(axis=1)
    columns = ["pruning", "propagation", "splitting", "collision_check", "repartitioning"]
    for_boxplot["total"] = for_boxplot[columns].sum(axis=1, numeric_only=True)
    columns.append("total")

    return create_boxplot(for_boxplot, columns, show_plot)


def create_boxplot(for_boxplot: pd.DataFrame, columns: List[str], show_plot: bool = False) -> List[
        Tuple[str, float, Tuple[float, float], Tuple[float, float]]]:
    _, bp = for_boxplot.boxplot(
        column=columns,
        showfliers=False,
        rot=45,
        return_type="both",
    )
    if show_plot:
        plt.show()
    # print boxplot data
    medians = [median.get_ydata()[0] for median in bp["medians"]]
    boxes = [(box.get_ydata()[0], box.get_ydata()[-2]) for box in bp["boxes"]]
    whiskers = [
        (lo.get_ydata()[-1], hi.get_ydata()[-1]) for lo, hi in zip(bp["whiskers"][0::2], bp["whiskers"][1::2])
    ]
    return list(zip(columns, medians, boxes, whiskers))


def read_otf_results(path: str) -> pd.DataFrame:
    with open(path) as f:
        data = yaml.load(f, Loader=yaml.FullLoader)
    name = data["scenario_name"]
    bench_results = [to_summary_result(*parse_benchmark_result(result), name) for result in data["results"]]
    return pd.DataFrame(bench_results)


def read_labeling_results(path: str) -> pd.DataFrame:
    with open(path) as f:
        data = yaml.load(f, Loader=yaml.FullLoader)
    name = data["scenario_name"]
    bench_results = [to_summary_result(*parse_benchmark_result(result), name, result["model_checking_time"]) for result in data["results"]]
    return pd.DataFrame(bench_results)


def parse_benchmark_result(result: dict) -> Tuple[ReachBenchmarkResults, float]:
    bench_result = ReachBenchmarkResults()
    bench_result.cnt_nodes_before_pruning = result["nodes_before_pruning"]
    bench_result.cnt_nodes_after_pruning = result["nodes_after_pruning"]
    bench_result.automaton_creation_time = result["automaton_creation_time"]
    bench_result.other_initialization_time = result["other_initialization_time"]
    bench_result.pruning_time = result["pruning_time"]

    steps = result["steps"]
    comp_times = result["computation_times"]
    comp_time_keys = result["computation_time_keys"]

    bench_result.computation_times_per_step = {
        step: ReachComputationTimes(**{k: v for k, v in zip(comp_time_keys, comp_time)})
        for step, comp_time in zip(steps, comp_times)
    }
    return bench_result, result["overall_time"]


def to_summary_result(benchmark_result: ReachBenchmarkResults, overall_time: float, name: str, model_checking_time: Optional[float] = None) -> SummaryBenchmarkResult:
    return SummaryBenchmarkResult(
        scenario_name=name,
        automaton_creation=benchmark_result.automaton_creation_time,
        other_initialization=benchmark_result.other_initialization_time,
        pruning=benchmark_result.pruning_time,
        propagation=benchmark_result.total_propagation_time,
        splitting=benchmark_result.total_splitting_time,
        partitioning=benchmark_result.total_partitioning_time,
        collision_check=benchmark_result.total_collision_check_time,
        merge=benchmark_result.total_merge_time,
        node_creation=benchmark_result.total_node_creation_time,
        overall=overall_time,
        nodes_before_pruning=benchmark_result.cnt_nodes_before_pruning,
        nodes_after_pruning=benchmark_result.cnt_nodes_after_pruning,
        model_checking=model_checking_time,
    )
