import os
from typing import Iterator

from benchmark import benchmark_with_progress, run_scenario

from analysis import otf_labeling_comparison, boxplot_computation_times_otf


def main():
    # reproduce_table_1()
    # otf_labeling_comparison(os.path.join(this_dir(), "data_table_1_cpp"))
    # otf_labeling_comparison(os.path.join(this_dir(), "data_table_1_python"))
    # reproduce_figure_4()
    # boxplot_computation_times_otf(os.path.join(this_dir(), "data_figure_4"), show_plot=True)
    reproduce_figure_3()


def reproduce_figure_3():
    # run YIELD scenario normally
    name = "ZAM_Yield-1_1_T-1"
    run_scenario(name, otf=True, cpp=True, draw=True, path_root=this_dir())
    run_scenario(name, otf=False, cpp=True, draw=True, path_root=this_dir())


def reproduce_table_1():
    scenario_names = ["ZAM_Yield-1_1_T-1", "ZAM_Merge-1_1_T-1", "ZAM_Intersection-1_2_T-1"]
    benchmark_with_progress(scenario_names, 0, repetitions=5, cpp=True,
                            path_root=this_dir(),
                            output_dir="data_table_1_cpp")
    benchmark_with_progress(scenario_names, 0, repetitions=5, cpp=False,
                            path_root=this_dir(),
                            output_dir="data_table_1_python")


def reproduce_figure_4():
    scenario_names = list(scenarios_from_file("exiD.txt"))
    benchmark_with_progress(scenario_names, 1, repetitions=5, cpp=True,
                            path_root=this_dir(),
                            output_dir="data_figure_4")


def this_dir() -> str:
    return os.path.abspath(os.path.dirname(__file__))


def scenarios_from_file(path: str) -> Iterator[str]:
    with open(path) as f:
        for line in f:
            yield line.strip()


if __name__ == "__main__":
    main()
