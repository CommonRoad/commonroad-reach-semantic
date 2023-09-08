import os
import shutil
from typing import Iterator, Tuple

from analysis import otf_labeling_comparison, boxplot_computation_times_otf
from benchmark import benchmark_with_progress, run_scenario


def main():
    # reproduce_table_1()
    # otf_labeling_comparison(os.path.join(this_dir(), "data_table_1_cpp"))
    # otf_labeling_comparison(os.path.join(this_dir(), "data_table_1_python"))
    reproduce_figure_4()
    # boxplot_computation_times_otf(os.path.join(this_dir(), "data_figure_4"), show_plot=True)
    # reproduce_figure_3()


def reproduce_figure_3():
    name = "ZAM_Yield-1_1_T-1"
    run_scenario(name, otf=True, cpp=True, draw=True, path_root=this_dir())
    run_scenario(name, otf=False, cpp=True, draw=True, path_root=this_dir())

    output_dir = os.path.join(this_dir(), "output")
    step = 9
    figures = [
        ("fig_3a", os.path.join(output_dir, f"{name}.cpp.otf", f"svgreach_{step:05d}.svg")),
        ("fig_3b", os.path.join(output_dir, f"{name}.cpp.labeling", f"svgreach_{step:05d}.svg")),
        ("fig_3b_hatching", os.path.join(output_dir, f"{name}.cpp.labeling", f"svgkripke_{step:05d}.svg")),
    ]
    for name, path in figures:
        shutil.copy(path, os.path.join(this_dir(), f"{name}.svg"))
    print(f"Figure 3 written to {this_dir()}")


def reproduce_table_1():
    intersection_scenarios = ["ZAM_Yield-1_1_T-1", "ZAM_Intersection-1_2_T-1"]
    interstate_scenarios = ["ZAM_Merge-1_1_T-1"]
    scenario_names = intersection_scenarios + interstate_scenarios
    benchmark_with_progress(scenario_names, 0, repetitions=5, cpp=True,
                            path_root=this_dir(),
                            output_dir="data_table_1_cpp")
    benchmark_with_progress(scenario_names, 0, repetitions=5, cpp=False,
                            path_root=this_dir(),
                            output_dir="data_table_1_python")

    cpp_comparison = otf_labeling_comparison(os.path.join(this_dir(), "data_table_1_cpp"))
    python_comparison = otf_labeling_comparison(os.path.join(this_dir(), "data_table_1_python"))

    table = cpp_comparison[[
        "automaton_creation",
        "nodes_before_pruning_otf",
        "nodes_before_pruning_lab",
        "computation_otf",
        "computation_lab"
    ]].join(python_comparison[["pure_model_checking"]])

    table.rename(inplace=True, columns={
        "nodes_before_pruning_otf": "base_sets_otf",
        "nodes_before_pruning_lab": "base_sets_labeling",
        "computation_otf": "computation_time_otf",
        "computation_lab": "computation_time_labeling",
        "pure_model_checking": "model_checking",
    })
    table.index.names = ["scenario_id"]
    table = table.astype(dtype={"base_sets_otf": int, "base_sets_labeling": int})

    latex_lines = [
                      r"\begin{tabular}{@{}lrrrrr@{}}",
                      r"    \toprule",
                      r"    \multicolumn{1}{c}{ID (\texttt{ZAM\_} \dots)} & \multicolumn{2}{c}{\#Base Sets} && \multicolumn{2}{c}{Comp. time} \\",
                      r"    \cmidrule{2-3} \cmidrule{5-6}",
                      r"     & \multicolumn{1}{c}{Ours} & \multicolumn{1}{c}{Theirs} && \multicolumn{1}{c}{Ours} & \multicolumn{1}{c}{Theirs} \\",
                      r"    \midrule",

                  ] + [
                      f"    {latex_command('texttt', idx)} & " +
                      f"{latex_command('num', int(row['base_sets_otf']))} & " +
                      f"{latex_command('num', int(row['base_sets_labeling']))} && " +
                      f"{latex_command('qty', round(row['computation_time_otf'] * 1000), latex_command('ms'))} & " +
                      f"{latex_command('qty', round(row['computation_time_labeling'] * 1000), latex_command('ms'))} \\\\"
                      for idx, row in table.rename(index={
                            "ZAM_Yield-1_1_T-1": "Yield-1_1_T-1",
                            "ZAM_Intersection-1_2_T-1": "TIV-2_1_T-1",
                            "ZAM_Merge-1_1_T-1": "TIV-1_1_T-1",
                        }).iterrows()
                  ] + [
                      r"    \bottomrule",
                      r"\end{tabular}",
                  ]
    automaton_creation_interstate = table.loc[interstate_scenarios, "automaton_creation"].mean()
    automaton_creation_intersection = table.loc[intersection_scenarios, "automaton_creation"].mean()

    filename = os.path.join(this_dir(), "table_1.tex")
    with open(filename, "w") as f:
        f.write("\n".join(latex_lines))
        f.write("\n")
        f.write(f"Automaton creation interstate scenarios: {round(automaton_creation_interstate * 1000)} ms\n")
        f.write(f"Automaton creation intersection scenarios: {round(automaton_creation_intersection * 1000)} ms\n")
        f.write(f"Average model checking overhead: {round(table['model_checking'].mean() * 1000)} ms\n")
    print(f"Written Table 1 to {filename}")


def reproduce_figure_4():
    scenario_names = list(scenarios_from_file("exiD.txt"))
    benchmark_with_progress(scenario_names, 1, repetitions=5, cpp=True,
                            path_root=this_dir(),
                            output_dir="data_figure_4")

    bp = boxplot_computation_times_otf(os.path.join(this_dir(), "data_figure_4"), show_plot=False)

    def to_ms(seconds: float) -> float:
        return round(seconds * 1000, 3)

    latex_plot = "\n".join(
        pgfplots_boxplot(column, to_ms(median), (to_ms(lb), to_ms(ub)), (to_ms(lw), to_ms(uw))) + "\n"
        for column, median, (lb, ub), (lw, uw) in bp
    )

    filename = os.path.join(this_dir(), "figure_4.tex")
    with open(filename, "w") as f:
        f.write(latex_plot)
        f.write("\n")
    print(f"Written Figure 4 to {filename}")


def this_dir() -> str:
    return os.path.abspath(os.path.dirname(__file__))


def scenarios_from_file(path: str) -> Iterator[str]:
    with open(path) as f:
        for line in f:
            yield line.strip()


def pgfplots_boxplot(name: str, median: float, box: Tuple[float, float], whisker: Tuple[float, float]) -> str:
    lines = [
        f"% {name}",
        r"\addplot+[boxplot prepared={",
        f"    lower whisker={whisker[0]}, lower quartile={box[0]},",
        f"    median={median},",
        f"    upper quartile={box[1]}, upper whisker={whisker[1]},",
        r"}] coordinates {};",
    ]
    return "\n".join(lines)

def latex_command(command: str, *args) -> str:
    latex_args = "".join("{" + str(arg).replace("_", "\\_") + "}" for arg in args)
    return f"\\{command}{latex_args}"


if __name__ == "__main__":
    main()
