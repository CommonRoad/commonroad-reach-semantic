import argparse
import glob
import os
import shutil
from typing import Iterator, Tuple, List

from analysis import otf_labeling_comparison, boxplot_computation_times_otf, boxplot_computation_times_labeling
from benchmark import benchmark_with_progress, run_scenario


def main(figure_3: bool = True, table_1: bool = True, figure_4: bool = True, exid_offline: bool = True,
         regenerate_data: bool = False):
    if figure_3:
        reproduce_figure_3(regenerate_data=regenerate_data)
    if table_1:
        reproduce_table_1(regenerate_data=regenerate_data)
    if figure_4:
        reproduce_figure_4(regenerate_data=regenerate_data)
    if exid_offline:
        exid_boxplot_offline(regenerate_data=regenerate_data)


def reproduce_figure_3(regenerate_data: bool = False):
    name = "ZAM_Yield-1_1_T-1"

    output_dir = "output"
    otf_output_dir = os.path.join(output_dir, f"{name}.cpp.otf")
    otf_output_dir_no_prune = os.path.join(output_dir, f"{name}.cpp.otf.no_prune")
    labeling_output_dir = os.path.join(output_dir, f"{name}.cpp.labeling")

    if not regenerate_data and (not os.path.exists(os.path.join(this_dir(), otf_output_dir)) or
                                not os.path.exists(os.path.join(this_dir(), otf_output_dir_no_prune)) or
                                not os.path.exists(os.path.join(this_dir(), labeling_output_dir))):
        print(f"No data for Figure 3 found. Regenerating data...")
        regenerate_data = True

    if regenerate_data:
        if not delete_output_dir_if_exists(otf_output_dir):
            return

        if not delete_output_dir_if_exists(otf_output_dir_no_prune):
            return

        if not delete_output_dir_if_exists(labeling_output_dir):
            return

        run_scenario(name, otf=True, cpp=True, draw=True, path_root=this_dir())
        run_scenario(name, otf=True, cpp=True, draw=True, prune=False, path_root=this_dir())
        run_scenario(name, otf=False, cpp=True, draw=True, path_root=this_dir())

    step = 9
    figures = [
        ("fig_3a", os.path.join(this_dir(), otf_output_dir, f"svgreach_{step:05d}.svg")),
        ("fig_3b", os.path.join(this_dir(), labeling_output_dir, f"svgreach_{step:05d}.svg")),
        ("fig_3b_hatching", os.path.join(this_dir(), labeling_output_dir, f"svgkripke_{step:05d}.svg")),
    ]
    for name, path in figures:
        shutil.copy(path, os.path.join(this_dir(), f"{name}.svg"))
    print(f"Figure 3 written to {this_dir()}")

    # copy images for video
    video_sections = [
        ("otf_no_prune", otf_output_dir_no_prune, "svgreach_*.svg"),
        ("otf", otf_output_dir, "svgreach_*.svg"),
        ("labeling", labeling_output_dir, "svgreach_*.svg"),
        ("labeling_model_checked", labeling_output_dir, "svgkripke_*.svg"),
    ]
    video_dir = os.path.join(this_dir(), "video")
    if not os.path.exists(video_dir):
        os.mkdir(video_dir)

    for name, path, pattern in video_sections:
        section_dir = os.path.join(video_dir, name)
        if not os.path.exists(section_dir):
            os.mkdir(section_dir)
        for frame in glob.glob(os.path.join(path, pattern)):
            shutil.copy(frame, section_dir)
    print(f"Video frames written to {video_dir}")
    print("Run make_video.sh to create the video")


def reproduce_table_1(regenerate_data: bool = False):
    intersection_scenarios = ["ZAM_Yield-1_1_T-1", "ZAM_Intersection-1_2_T-1"]
    interstate_scenarios = ["ZAM_Merge-1_1_T-1"]
    scenario_names = intersection_scenarios + interstate_scenarios

    cpp_output_dir = "data_table_1_cpp"
    python_output_dir = "data_table_1_python"

    if not regenerate_data and (not os.path.exists(os.path.join(this_dir(), cpp_output_dir)) or not os.path.exists(
            os.path.join(this_dir(), python_output_dir))):
        print(f"No data for Table 3 found. Regenerating data...")
        regenerate_data = True

    if regenerate_data:
        if not delete_output_dir_if_exists(cpp_output_dir):
            return

        if not delete_output_dir_if_exists(python_output_dir):
            return

        benchmark_with_progress(scenario_names, 0, repetitions=5, cpp=True,
                                path_root=this_dir(),
                                output_dir=cpp_output_dir)
        benchmark_with_progress(scenario_names, 0, repetitions=5, cpp=False,
                                path_root=this_dir(),
                                output_dir=python_output_dir)

    cpp_comparison = otf_labeling_comparison(os.path.join(this_dir(), cpp_output_dir))
    python_comparison = otf_labeling_comparison(os.path.join(this_dir(), python_output_dir))

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
    print(f"Table 1 written to {filename}")


def reproduce_figure_4(regenerate_data: bool = False):
    scenario_names = list(scenarios_from_file("exiD.txt"))

    output_dir = "data_figure_4"

    if not regenerate_data and not os.path.exists(os.path.join(this_dir(), output_dir)):
        print(f"No data for Figure 4 found. Regenerating data...")
        regenerate_data = True

    if regenerate_data:
        if not delete_output_dir_if_exists(output_dir):
            return
        benchmark_with_progress(scenario_names, 1, repetitions=5, cpp=True,
                                path_root=this_dir(),
                                output_dir=output_dir)

    comp_times, bp = boxplot_computation_times_otf(os.path.join(this_dir(), output_dir), show_plot=False)

    high_total_times = comp_times[comp_times["total"] > 0.1].sort_values("total", ascending=False)

    latex = latex_plot(bp)

    filename = os.path.join(this_dir(), "figure_4.tex")
    with open(filename, "w") as f:
        f.write(latex)
        f.write("\n")
        for idx, row in high_total_times.iterrows():
            total = row["total"]
            f.write(f"{idx}: {round(total * 1000)} ms\n")
    print(f"Figure 4 written to {filename}")


def exid_boxplot_offline(regenerate_data: bool = False):
    scenario_names = list(scenarios_from_file("exiD.txt"))

    output_dir = "data_exid_boxplot_offline"

    if not regenerate_data and not os.path.exists(os.path.join(this_dir(), output_dir)):
        print(f"No data for exiD labeling found. Regenerating data...")
        regenerate_data = True

    if regenerate_data:
        if not delete_output_dir_if_exists(output_dir):
            return
        benchmark_with_progress(scenario_names, 2, repetitions=5, cpp=True,
                                path_root=this_dir(),
                                output_dir=output_dir)

    _, bp = boxplot_computation_times_labeling(os.path.join(this_dir(), output_dir), show_plot=False)

    latex = latex_plot(bp)

    filename = os.path.join(this_dir(), "exid_boxplot_offline.tex")
    with open(filename, "w") as f:
        f.write(latex)
        f.write("\n")
    print(f"Boxplot exiD offline written to {filename}")


def this_dir() -> str:
    return os.path.abspath(os.path.dirname(__file__))


def scenarios_from_file(path: str) -> Iterator[str]:
    with open(path) as f:
        for line in f:
            yield line.strip()


def latex_plot(boxplot_data: List[Tuple[str, float, Tuple[float, float], Tuple[float, float]]]) -> str:
    def to_ms(seconds: float) -> float:
        return round(seconds * 1000, 3)

    return "\n".join(
        pgfplots_boxplot(column, to_ms(median), (to_ms(lb), to_ms(ub)), (to_ms(lw), to_ms(uw))) + "\n"
        for column, median, (lb, ub), (lw, uw) in boxplot_data
    )


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


def delete_output_dir_if_exists(output_dir: str) -> bool:
    if os.path.exists(os.path.join(this_dir(), output_dir)):
        print(f"WARNING: Output directory {output_dir} already exists.")
        return interactive_delete(os.path.join(this_dir(), output_dir))
    return False


def interactive_delete(path: str) -> bool:
    if os.path.exists(path):
        if input(f"Delete {path}? [y/N] ").lower() == "y":
            shutil.rmtree(path)
            return True
        return False
    return True


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Reproduce results from the paper. If no specific figure or table is given, all are reproduced."
    )
    parser.add_argument("--figure-3", action="store_true", help="create Figure 3")
    parser.add_argument("--table-1", action="store_true", help="create Table 1")
    parser.add_argument("--figure-4", action="store_true", help="create Figure 4")
    parser.add_argument("--exid-offline", action="store_true", help="create boxplot for exiD with offline approach")
    parser.add_argument("--regenerate-data", action="store_true", help="force data regeneration")
    args = parser.parse_args()
    # if no switch is given, run all
    if not any((args.figure_3, args.table_1, args.figure_4, args.exid_offline)):
        main(regenerate_data=args.regenerate_data)
    else:
        main(figure_3=args.figure_3, table_1=args.table_1, figure_4=args.figure_4, exid_offline=args.exid_offline,
             regenerate_data=args.regenerate_data)
