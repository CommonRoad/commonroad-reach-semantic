import glob
import os
from dataclasses import dataclass
from typing import Tuple

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


def main():
    path_root = "/home/lercher/datasets/exiD-commonroad-only6-merge"
    benchmark_dir = "benchmark_scenarios_starting_in_front"
    scenario_paths = glob.glob(os.path.join(path_root, benchmark_dir, "O_DEU_MerzenichRather-*.yaml"))
    results = pd.concat((read_otf_results(path) for path in scenario_paths), ignore_index=True, sort=False)
    aggregated_per_scenario = results.groupby("scenario_name").mean()
    overall_mean = aggregated_per_scenario.mean()
    for_boxplot = results.drop(columns=["other_initialization", "overall", "nodes_before_pruning", "nodes_after_pruning"])
    for_boxplot["total"] = for_boxplot.sum(axis=1, numeric_only=True)
    for_boxplot.boxplot(
        column=[
            "automaton_creation", "pruning", "propagation",
            "splitting", "partitioning", "collision_check", "merge",
            "node_creation", "total",
        ],
        showfliers=False,
        rot=45,
    )
    plt.show()
    print(overall_mean)


def read_otf_results(path: str) -> pd.DataFrame:
    with open(path) as f:
        data = yaml.load(f, Loader=yaml.FullLoader)
    name = data["scenario_name"]
    bench_results = [to_summary_result(*parse_benchmark_result(result), name) for result in data["results"]]
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


def to_summary_result(benchmark_result: ReachBenchmarkResults, overall_time: float, name: str) -> SummaryBenchmarkResult:
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
    )


if __name__ == "__main__":
    main()
