#include "pybind.hpp"

#include "reach_semantic/benchmark/benchmark_result.hpp"

namespace py = pybind11;
using namespace semantic_reach;

void export_benchmark(py::module &m) {
    py::class_<ReachBenchmarkResults>(m, "ReachBenchmarkResults")
            .def(py::init<>())
            .def_readonly("computation_times_per_step", &ReachBenchmarkResults::computation_times_per_step)
            .def_property_readonly("automaton_creation_time", [](const ReachBenchmarkResults &r) {
                return static_cast<double>(r.automaton_creation_time) / 1'000'000;
            })
            .def_readonly("cnt_nodes_before_pruning", &ReachBenchmarkResults::cnt_nodes_before_pruning)
            .def_readonly("cnt_nodes_after_pruning", &ReachBenchmarkResults::cnt_nodes_after_pruning);

    py::class_<ReachComputationTimes>(m, "ReachComputationTimes")
            .def(py::init<>())
            .def_property_readonly("propagation", [](const ReachComputationTimes &r) {
                return static_cast<double>(r.propagation) / 1'000'000;
            })
            .def_property_readonly("splitting", [](const ReachComputationTimes &r) {
                return static_cast<double>(r.splitting) / 1'000'000;
            })
            .def_property_readonly("collision_check", [](const ReachComputationTimes &r) {
                return static_cast<double>(r.collision_check) / 1'000'000;
            })
            .def_property_readonly("node_creation", [](const ReachComputationTimes &r) {
                return static_cast<double>(r.node_creation) / 1'000'000;
            });
}
