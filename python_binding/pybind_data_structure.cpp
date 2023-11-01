#include "pybind.hpp"

namespace py = pybind11;
using namespace semantic_reach;

void export_data_structures(py::module &module) {
    export_semantic_model(module);
    export_traffic_rule_interface(module);
    export_configuration(module);
}

void export_semantic_model(py::module &module) {
    py::class_<SemanticModel, shared_ptr<SemanticModel>>(module, "SemanticModel")
        .def(py::init<py::object const &>(), py::arg("semantic_model_py"))
        .def_readonly("step_start", &SemanticModel::step_start)
        .def_readonly("step_end", &SemanticModel::step_end)
        .def_readonly("vec_regions", &SemanticModel::vec_regions)
        .def_readonly("map_step_to_position_intervals", &SemanticModel::map_step_to_position_intervals);

    py::class_<Region, shared_ptr<Region>>(module, "Region")
        .def(py::init<>())
        .def(py::init<py::object const &>(), py::arg("region_py"))
        .def_readonly("step_end", &Region::step_end)
        .def_readonly("set_ids_lanelets", &Region::set_ids_lanelets)
        .def_readonly("polygon_cart", &Region::polygon_cart)
        .def_readonly("polygon_cvln", &Region::polygon_cvln)
        .def("map_group_to_propositions_at_step", &Region::map_group_to_propositions_at_step);

    py::class_<PropositionHolder, shared_ptr<PropositionHolder>>(module, "PropositionHolder")
        .def(py::init<>())
        .def_readwrite("set_propositions", &PropositionHolder::set_propositions)
        .def_readwrite("set_propositions_temporary", &PropositionHolder::set_propositions_temporary)
        .def("propositions", &PropositionHolder::propositions, py::arg("include_temporary") = true)
        .def("add_propositions",
             static_cast<void (PropositionHolder::*)(set<string> const &, PropositionGroup const &)>(
                 &PropositionHolder::add_propositions))
        .def("add_propositions", static_cast<void (PropositionHolder::*)(set<string> const &, py::handle const &)>(
                                     &PropositionHolder::add_propositions))
        .def("propositions_in_group", static_cast<set<string> (PropositionHolder::*)(PropositionGroup const &)>(
                                          &PropositionHolder::propositions_in_group))
        .def("propositions_in_group", static_cast<set<string> (PropositionHolder::*)(py::handle const &)>(
                                          &PropositionHolder::propositions_in_group))
        .def("add_proposition", &PropositionHolder::add_proposition, py::arg("proposition"), py::arg("group"));

    py::enum_<PropositionGroup>(module, "PropositionGroup")
        .value("POSITION", PropositionGroup::POSITION)
        .value("VELOCITY", PropositionGroup::VELOCITY)
        .value("ACCELERATION", PropositionGroup::ACCELERATION)
        .value("VEHICLE", PropositionGroup::VEHICLE)
        .value("TRAFFIC_SIGN", PropositionGroup::TRAFFIC_SIGN)
        .value("TRAFFIC_LIGHT", PropositionGroup::TRAFFIC_LIGHT)
        .value("INTERSECTION", PropositionGroup::INTERSECTION)
        .value("PRIORITY", PropositionGroup::PRIORITY)
        .value("TRAFFIC_STATUS", PropositionGroup::TRAFFIC_STATUS)
        .value("TEMPORARY", PropositionGroup::TEMPORARY)
        .export_values();
}

void export_traffic_rule_interface(py::module &module) {
    py::class_<TrafficRuleInterface, shared_ptr<TrafficRuleInterface>>(module, "TrafficRuleInterface")
        .def(py::init<py::object const &>(), py::arg("traffic_rule_interface_py"));
}

void export_configuration(py::module &module) {

    py::class_<SemanticConfiguration, shared_ptr<SemanticConfiguration>, reach::Configuration>(module,
                                                                                               "SemanticConfiguration")
        .def(py::init<>())
        .def_readwrite("reachable_set", &SemanticConfiguration::config_reachable_set)
        .def_readwrite("traffic_rule", &SemanticConfiguration::config_traffic_rule)
        .def_readwrite("semantic_model", &SemanticConfiguration::config_semantic_model);

    py::class_<TrafficRuleConfiguration, shared_ptr<TrafficRuleConfiguration>>(module, "TrafficRuleConfiguration")
        .def(py::init<>())
        .def_readwrite("distance_braking", &TrafficRuleConfiguration::distance_braking)
        .def_readwrite("acceleration_braking_hard", &TrafficRuleConfiguration::acceleration_braking_hard)
        .def_readwrite("backward_driving_v_err", &TrafficRuleConfiguration::backward_driving_v_err)
        .def_readwrite("dis_stop_line", &TrafficRuleConfiguration::dis_stop_line)
        .def_readwrite("activated_rules", &TrafficRuleConfiguration::activated_rules)
        .def_readwrite("mode_spot", &TrafficRuleConfiguration::mode_spot)
        .def_readwrite("mode_automata", &TrafficRuleConfiguration::mode_automata);

    py::class_<SemanticModelConfiguration, shared_ptr<SemanticModelConfiguration>>(module, "SemanticModelConfiguration")
        .def(py::init<>())
        .def_readwrite("is_intersection", &SemanticModelConfiguration::is_intersection)
        .def_readwrite("ego_radius_inflation", &SemanticModelConfiguration::ego_radius_inflation)
        .def_readwrite("vec_route_lanelet_ids", &SemanticModelConfiguration::vec_route_lanelet_ids);
}