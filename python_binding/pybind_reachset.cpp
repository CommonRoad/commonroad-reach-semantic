#include "pybind.hpp"

namespace py = pybind11;
using namespace semantic_reach;

void export_reach(py::module &m) {
    export_reachable_set_interface(m);
    export_reachable_set_labeler(m);
}

void export_reachable_set_interface(py::module &m) {
    py::class_<SemanticLabelingReachableSet, shared_ptr<SemanticLabelingReachableSet>>(m, "SemanticLabelingReachableSet")
            .def(py::init<SemanticConfigurationPtr const &, CollisionCheckerPtr const &,
                         SemanticModelPtr const &, TrafficRuleInterfacePtr const &>(),
                 py::arg("configuration"),
                 py::arg("collision_checker"),
                 py::arg("semantic_model"),
                 py::arg("traffic_rule_interface"))
            .def_readonly("step_start", &SemanticLabelingReachableSet::step_start)
            .def_readonly("step_end", &SemanticLabelingReachableSet::step_end)
            .def_readonly("map_step_to_propositions_to_drivable_area",
                          &SemanticLabelingReachableSet::map_step_to_propositions_to_drivable_area)
            .def_readonly("config", &SemanticLabelingReachableSet::config)
            .def_readonly("labeler", &SemanticLabelingReachableSet::labeler)
            .def("compute", &SemanticLabelingReachableSet::compute, py::arg("step_start") = 1, py::arg("step_end") = 0)
            .def("drivable_area_at_step", &SemanticLabelingReachableSet::drivable_area_at_step, py::arg("step"))
            .def("reachable_set_at_step", &SemanticLabelingReachableSet::reachable_set_at_step, py::arg("step"))
            .def("drivable_area", &SemanticLabelingReachableSet::drivable_area)
            .def("reachable_set", &SemanticLabelingReachableSet::reachable_set)
            .def("propagated_set", &SemanticLabelingReachableSet::propagated_set);
}

void export_reachable_set_labeler(py::module &m) {
    py::class_<ReachableSetLabeler, shared_ptr<ReachableSetLabeler>>(m, "ReachableSetLabeler")
            .def(py::init<SemanticModelPtr, SemanticConfigurationPtr>(), py::arg("semantic_model"), py::arg("config"))
            .def_readonly("reachable_set_to_propositions", &ReachableSetLabeler::reachable_set_to_propositions)
            .def_readonly("reachable_set_to_lanelet_ids", &ReachableSetLabeler::reachable_set_to_lanelet_ids);
}
