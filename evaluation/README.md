# Comparison Plots

* `ZAM_plots`: plots for the 3 scenarios using the on-the-fly and labeling approaches
* `ZAM_benchmark`: C++ computation times for both approches
* `ZAM_benchmark_python`: Python computation times for both approaches

* Computation times reported in paper come from `ZAM_benchmark`
    * Exception: Model checking step for labeling approach is from `ZAM_benchmark_python`, as the measured time in C++ includes overhead of copying data between Python and C++
* Plots shown in paper:
    * On-the-fly approach: `ZAM_plots/ZAM_Yield-1_1_T-1.otf/svgreach_00009.svg`
    * Labeling approach: `ZAM_plots/ZAM_Yield-1_1_T-1.labeling/svgreach_00009.svg` (with hatched area corresponding to `ZAM_plots/ZAM_Yield_1_1_T-1.labeling/svgkripke_00009.svg`)

# exiD Benchmark

All benchmarks are for the C++ implementation.

* `exiD_benchmark`: data for on-the-fly approach
* `exiD_benchmark_labeling`: data for labeling approach
* `exiD_benchmark_both`: data for both approaches (measured together, in every experiment on-the-fly runs before labeling)

* Boxplot for the on-the-fly approach (in paper) shows the data from `exiD_benchmark`
* Boxplot for the labeling approach (not in paper) shows the data from `exiD_benchmark_both`

* The boxplots report the benchmark with the lower computation time for each approach
* I don't know why labeling was faster when we measured both approaches together, while on-the-fly was slower in this case

# Notes

* In the paper, `ZAM_Merge-1_1_T-1` is called `ZAM_TIV-1_1_T-1` and `ZAM_Intersection-1_2_T-1` is called `ZAM_TIV-2_1_T-1`
* For the labeling approach the time for translating the specification into an automaton is attributed to the model checking time
    * To obtain the extra overhead for model checking reported in the paper, we subtracted the automaton creation time of the on-the-fly approach from the measured model checking time of the labeling approach
    * This is fine, as both approaches compute the same automaton with the same tool (spot)

