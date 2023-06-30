#include "pybind.hpp"

namespace py = pybind11;
using namespace semantic_reach;

void export_reach(py::module &m) {
    export_reachable_set_interface(m);
    export_reachable_set_labeler(m);
}

void export_reachable_set_interface(py::module &m) {
    py::class_<SemanticReachableSet, shared_ptr<SemanticReachableSet>>(m, "SemanticReachableSet")
            .def_readonly("step_start", &SemanticReachableSet::step_start)
            .def_readonly("step_end", &SemanticReachableSet::step_end)
            .def_readonly("config", &SemanticReachableSet::config)
            .def_readonly("labeler", &SemanticReachableSet::labeler)
            .def("compute", &SemanticReachableSet::compute, py::arg("step_start") = 1, py::arg("step_end") = 0)
            .def("drivable_area_at_step", &SemanticReachableSet::drivable_area_at_step, py::arg("step"))
            .def("reachable_set_at_step", &SemanticReachableSet::reachable_set_at_step, py::arg("step"))
            .def("drivable_area", &SemanticReachableSet::drivable_area)
            .def("reachable_set", &SemanticReachableSet::reachable_set)
            .def("propagated_set", &SemanticReachableSet::propagated_set)
            .def("prune_nodes_not_reaching_final_step", &SemanticReachableSet::prune_nodes_not_reaching_final_step);

    py::class_<SemanticLabelingReachableSet, shared_ptr<SemanticLabelingReachableSet>, SemanticReachableSet>(m, "SemanticLabelingReachableSet")
            .def(py::init<SemanticConfigurationPtr const &, CollisionCheckerPtr const &,
                         SemanticModelPtr const &, TrafficRuleInterfacePtr const &>(),
                 py::arg("configuration"),
                 py::arg("collision_checker"),
                 py::arg("semantic_model"),
                 py::arg("traffic_rule_interface"))
            .def_readonly("map_step_to_propositions_to_drivable_area",
                          &SemanticLabelingReachableSet::map_step_to_propositions_to_drivable_area);

    py::class_<SemanticOTFReachableSet, shared_ptr<SemanticOTFReachableSet>, SemanticReachableSet>(m, "SemanticOTFReachableSet")
            .def(py::init<SemanticConfigurationPtr const &, CollisionCheckerPtr const &,
                         SemanticModelPtr const &, TrafficRuleInterfacePtr const &>(),
                 py::arg("configuration"),
                 py::arg("collision_checker"),
                 py::arg("semantic_model"),
                 py::arg("traffic_rule_interface"))
            .def_readonly("reachable_set_to_label", &SemanticOTFReachableSet::reachable_set_to_label);

    py::class_<SemanticSplittingOTFReachableSet, shared_ptr<SemanticSplittingOTFReachableSet>, SemanticReachableSet>(m, "SemanticSplittingOTFReachableSet")
            .def(py::init<SemanticConfigurationPtr const &, CollisionCheckerPtr const &,
                         SemanticModelPtr const &, TrafficRuleInterfacePtr const &>(),
                 py::arg("configuration"),
                 py::arg("collision_checker"),
                 py::arg("semantic_model"),
                 py::arg("traffic_rule_interface"))
             .def_readonly("reachable_set_to_label", &SemanticSplittingOTFReachableSet::reachable_set_to_label);
}

void export_reachable_set_labeler(py::module &m) {
    py::class_<ReachableSetLabeler, shared_ptr<ReachableSetLabeler>>(m, "ReachableSetLabeler")
            .def(py::init<SemanticModelPtr, SemanticConfigurationPtr>(), py::arg("semantic_model"), py::arg("config"))
            .def_readonly("reachable_set_to_propositions", &ReachableSetLabeler::reachable_set_to_propositions)
            .def_readonly("reachable_set_to_lanelet_ids", &ReachableSetLabeler::reachable_set_to_lanelet_ids);
}
