#include "pybind.hpp"

namespace py = pybind11;
using namespace semantic_reach;

void export_reach(py::module& m) {
    export_reachable_set_interface(m);
}

void export_reachable_set_interface(py::module& m) {
    py::class_<SemanticReachableSet, shared_ptr<SemanticReachableSet>>(m, "SemanticReachableSet")
            .def(py::init<SemanticConfigurationPtr&>(),
                 py::arg("configuration"))
            .def(py::init<SemanticConfigurationPtr const&, CollisionCheckerPtr const&, SemanticModelPtr const&>(),
                 py::arg("configuration"),
                 py::arg("collision_checker"),
                 py::arg("semantic_model"))
            .def(py::init<SemanticConfigurationPtr const&, CollisionCheckerPtr const&,
                         SemanticModelPtr const&, TrafficRuleInterfacePtr const&>(),
                 py::arg("configuration"),
                 py::arg("collision_checker"),
                 py::arg("semantic_model"),
                 py::arg("traffic_rule_interface"))
            .def_readonly("step_start", &SemanticReachableSet::step_start)
            .def_readonly("step_end", &SemanticReachableSet::step_end)
            .def_readonly("map_step_to_propositions_to_drivable_area",
                          &SemanticReachableSet::map_step_to_propositions_to_drivable_area)
            .def_readonly("map_step_to_propositions_to_reachable_set",
                          &SemanticReachableSet::map_step_to_propositions_to_reachable_set)
                    //.def_readwrite("map_time_to_reachable_set_refined",
                    //               &SemanticReachableSet::map_time_to_reachable_set_refined)
            .def_readonly("config", &SemanticReachableSet::config)
                    //.def("print_collision_checker_info", &SemanticReachableSet::print_collision_checker_info)
            .def("compute", &SemanticReachableSet::compute,
                 py::arg("step_start") = 1,
                 py::arg("step_end") = 0)
            .def("drivable_area_at_step", &SemanticReachableSet::drivable_area_at_step,
                 py::arg("step"))
            .def("drivable_area_merge_at_step", &SemanticReachableSet::drivable_area_merge_at_step,
                 py::arg("step"))
            .def("reachable_set_at_step", &SemanticReachableSet::reachable_set_at_step,
                 py::arg("step"))
            .def("reachable_set_merge_at_step", &SemanticReachableSet::reachable_set_merge_at_step,
                 py::arg("step"));
    //.def("refined_reachable_set_at_time_step", &SemanticReachableSet::refined_reachable_set_at_time_step,
    //     py::arg("time_step"))
    //.def("add_refined_node", &SemanticReachableSet::add_refined_node, py::arg("time_step"), py::arg("node"));
}