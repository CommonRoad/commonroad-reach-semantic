#include "pybind.hpp"

namespace py = pybind11;
using namespace reach;

void export_data_structures(py::module& m) {
    export_reach_polygon(m);
    export_reach_node(m);
    export_semantic_model(m);
    export_traffic_rule_interface(m);
    export_configuration(m);
}

void export_reach_polygon(py::module& m) {
    py::class_<ReachPolygon, shared_ptr<ReachPolygon >>(m, "ReachPolygon")
            .def(py::init<vector<tuple<double, double >> const&>(),
                 py::arg("vec_vertices"))
            .def("p_min", &ReachPolygon::p_min)
            .def("p_max", &ReachPolygon::p_max)
            .def("v_min", &ReachPolygon::v_min)
            .def("v_max", &ReachPolygon::v_max)
            .def("p_lon_min", &ReachPolygon::p_lon_min)
            .def("p_lon_max", &ReachPolygon::p_lon_max)
            .def("p_lat_min", &ReachPolygon::p_lat_min)
            .def("p_lat_max", &ReachPolygon::p_lat_max)
            .def("p_lon_center", &ReachPolygon::p_lon_center)
            .def("p_lat_center", &ReachPolygon::p_lat_center)
            .def("bounding_box", &ReachPolygon::bounding_box)
                    //.def("vertices", [](ReachPolygon const& polygon) {
                    //    py::list list_tuples_vertices;
                    //    for (auto const& vertex: polygon.vertices()) {
                    //        list_tuples_vertices.append(py::make_tuple(vertex.p_lon(), vertex.p_lat()));
                    //    }
                    //    return list_tuples_vertices;
                    //})
            .def("convexify", &ReachPolygon::convexify)
            .def("minkowski_sum", &ReachPolygon::minkowski_sum)
            .def("intersects", &ReachPolygon::intersects)
            .def("intersect_halfspace", &ReachPolygon::intersect_halfspace,
                 py::arg("a"), py::arg("b"), py::arg("c"))
            .def("intersects_rectangle", &ReachPolygon::intersects_rectangle,
                 py::arg("polygon_other"), py::arg("num_digits") = 2)
            .def("__repr__", [](ReachPolygon const& polygon) {
                return "(" + std::to_string(polygon.p_lon_min()) + ", " + std::to_string(polygon.p_lat_min())
                       + ", " + std::to_string(polygon.p_lon_max()) + ", " + std::to_string(polygon.p_lat_max()) + ")";
            });
}

void export_reach_node(py::module& m) {
    py::class_<ReachNode, shared_ptr<ReachNode >>(m, "ReachNode")
            .def(py::init<>())
            .def("p_lon_min", &ReachNode::p_lon_min)
            .def("p_lon_max", &ReachNode::p_lon_max)
            .def("p_lat_min", &ReachNode::p_lat_min)
            .def("p_lat_max", &ReachNode::p_lat_max)
            .def("v_lon_min", &ReachNode::v_lon_min)
            .def("v_lon_max", &ReachNode::v_lon_max)
            .def("v_lat_min", &ReachNode::v_lat_min)
            .def("v_lat_max", &ReachNode::v_lat_max)
            .def("position_rectangle", &ReachNode::position_rectangle)
            .def("set_propositions", &ReachNode::set_propositions, py::arg("include_temporary") = true)
            .def_readwrite("vec_nodes_parent", &ReachNode::vec_nodes_parent)
            .def_readwrite("vec_nodes_child", &ReachNode::vec_nodes_child)
            .def_readwrite("vec_nodes_source", &ReachNode::vec_nodes_source)
            .def("add_lanelet_ids", &ReachNode::add_lanelet_ids)
            .def_readonly("id", &ReachNode::id)
            .def_readonly("step", &ReachNode::step)
            .def_readonly("polygon_lon", &ReachNode::polygon_lon)
            .def_readonly("polygon_lat", &ReachNode::polygon_lat)
            .def_readwrite("proposition_holder", &ReachNode::proposition_holder)
            .def_readwrite("set_ids_lanelets", &ReachNode::set_ids_lanelets)
            .def("__repr__", [](ReachNode const& node) {
                return "(id:" + std::to_string(node.id) + ", "
                       + std::to_string(node.p_lon_min()) + ", " + std::to_string(node.p_lat_min())
                       + ", " + std::to_string(node.p_lon_max()) + ", " + std::to_string(node.p_lat_max()) + ")";
            })
            .def("__hash__", [](ReachNode const& node) {
                return py::hash(py::make_tuple(node.id, node.step));
            });
}

void export_semantic_model(py::module& m) {
    py::class_<SemanticModel, shared_ptr<SemanticModel >>(m, "SemanticModel")
            .def(py::init<py::object const&>(),
                 py::arg("semantic_model_py"))
            .def_readonly("step_start", &SemanticModel::step_start)
            .def_readonly("step_end", &SemanticModel::step_end)
            .def_readonly("vec_regions", &SemanticModel::vec_regions)
            .def_readonly("map_step_to_position_intervals", &SemanticModel::map_step_to_position_intervals);

    py::class_<Region, shared_ptr<Region >>(m, "Region")
            .def(py::init<>())
            .def(py::init<py::object const&>(),
                 py::arg("region_py"))
            .def_readonly("step_end", &Region::step_end)
            .def_readonly("set_ids_lanelets", &Region::set_ids_lanelets)
            .def_readonly("polygon_cart", &Region::polygon_cart)
            .def_readonly("polygon_cvln", &Region::polygon_cvln)
            .def("map_group_to_propositions_at_step", &Region::map_group_to_propositions_at_step);

    py::class_<PropositionHolder, shared_ptr<PropositionHolder >>(m, "PropositionHolder")
            .def(py::init<>())
            .def_readwrite("set_propositions", &PropositionHolder::set_propositions)
            .def_readwrite("set_propositions_temporary", &PropositionHolder::set_propositions_temporary)
            .def("propositions", &PropositionHolder::propositions, py::arg("include_temporary") = true)
            .def("add_propositions",
                 static_cast<void (PropositionHolder::*)
                         (set<string> const&, PropositionGroup const&)>(&PropositionHolder::add_propositions))
            .def("add_propositions",
                 static_cast<void (PropositionHolder::*)
                         (set<string> const&, py::handle const&)>(&PropositionHolder::add_propositions))
            .def("propositions_in_group",
                 static_cast<set<string> (PropositionHolder::*)
                         (PropositionGroup const&)>(&PropositionHolder::propositions_in_group))
            .def("propositions_in_group",
                 static_cast<set<string> (PropositionHolder::*)
                         (py::handle const&)>(&PropositionHolder::propositions_in_group))
            .def("add_proposition", &PropositionHolder::add_proposition,
                 py::arg("proposition"), py::arg("group"));

    py::enum_<PropositionGroup>(m, "PropositionGroup")
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

void export_traffic_rule_interface(py::module& m) {
    py::class_<TrafficRuleInterface, shared_ptr<TrafficRuleInterface >>(m, "TrafficRuleInterface")
            .def(py::init<py::object const&>(), py::arg("traffic_rule_interface_py"));
}

void export_configuration(py::module& m) {
    py::enum_<CoordinateSystem>(m, "CoordinateSystem")
            .value("CARTESIAN", CoordinateSystem::CARTESIAN)
            .value("CURVILINEAR", CoordinateSystem::CURVILINEAR)
            .export_values();

    py::enum_<ReferencePoint>(m, "ReferencePoint")
            .value("CENTER", ReferencePoint::CENTER)
            .value("REAR", ReferencePoint::REAR)
            .export_values();

    py::class_<Configuration, shared_ptr<Configuration >>(m, "Configuration")
            .def(py::init<>())
            .def_readwrite("general", &Configuration::config_general)
            .def_readwrite("vehicle", &Configuration::config_vehicle)
            .def_readwrite("planning", &Configuration::config_planning)
            .def_readwrite("reachable_set", &Configuration::config_reachable_set)
            .def_readwrite("debug", &Configuration::config_debug);

    py::class_<GeneralConfiguration, shared_ptr<GeneralConfiguration >>(m, "GeneralConfiguration")
            .def(py::init<>())
            .def_readwrite("name_scenario", &GeneralConfiguration::name_scenario)
            .def_readwrite("path_scenarios", &GeneralConfiguration::path_scenarios);

    py::class_<VehicleConfiguration, shared_ptr<VehicleConfiguration >>(m, "VehicleConfiguration")
            .def(py::init<>())
            .def_readwrite("ego", &VehicleConfiguration::ego)
            .def_readwrite("other", &VehicleConfiguration::other);

    py::class_<Ego, shared_ptr<Ego >>(m, "Ego")
            .def(py::init<>())
            .def_readwrite("id_type_vehicle", &Ego::id_type_vehicle)
            .def_readwrite("id_vehicle", &Ego::id_vehicle)
            .def_readwrite("length", &Ego::length)
            .def_readwrite("width", &Ego::width)
            .def_readwrite("radius_disc", &Ego::radius_disc)
            .def_readwrite("circle_distance", &Ego::circle_distance)
            .def_readwrite("wheelbase", &Ego::wheelbase)
            .def_readwrite("v_lon_min", &Ego::v_lon_min)
            .def_readwrite("v_lon_max", &Ego::v_lon_max)
            .def_readwrite("v_lat_min", &Ego::v_lat_min)
            .def_readwrite("v_lat_max", &Ego::v_lat_max)
            .def_readwrite("a_lon_min", &Ego::a_lon_min)
            .def_readwrite("a_lon_max", &Ego::a_lon_max)
            .def_readwrite("a_lat_min", &Ego::a_lat_min)
            .def_readwrite("a_lat_max", &Ego::a_lat_max)
            .def_readwrite("a_max", &Ego::a_max)
            .def_readwrite("t_react", &Ego::t_react)
            .def_readwrite("fov", &Ego::fov);

    py::class_<Other, shared_ptr<Other >>(m, "Other")
            .def(py::init<>())
            .def_readwrite("id_type_vehicle", &Other::id_type_vehicle)
            .def_readwrite("id_vehicle", &Other::id_vehicle)
            .def_readwrite("length", &Other::length)
            .def_readwrite("width", &Other::width)
            .def_readwrite("radius_disc", &Other::radius_disc)
            .def_readwrite("circle_distance", &Other::circle_distance)
            .def_readwrite("wheelbase", &Other::wheelbase)
            .def_readwrite("v_lon_min", &Other::v_lon_min)
            .def_readwrite("v_lon_max", &Other::v_lon_max)
            .def_readwrite("v_lat_min", &Other::v_lat_min)
            .def_readwrite("v_lat_max", &Other::v_lat_max)
            .def_readwrite("a_lon_min", &Other::a_lon_min)
            .def_readwrite("a_lon_max", &Other::a_lon_max)
            .def_readwrite("a_lat_min", &Other::a_lat_min)
            .def_readwrite("a_lat_max", &Other::a_lat_max)
            .def_readwrite("a_max", &Other::a_max)
            .def_readwrite("t_react", &Other::t_react);

    py::class_<PlanningConfiguration, shared_ptr<PlanningConfiguration >>(m, "PlanningConfiguration")
            .def(py::init<>())
            .def_readwrite("dt", &PlanningConfiguration::dt)
            .def_readwrite("step_start", &PlanningConfiguration::step_start)
            .def_readwrite("steps_computation", &PlanningConfiguration::steps_computation)
            .def_readwrite("p_lon_initial", &PlanningConfiguration::p_lon_initial)
            .def_readwrite("p_lat_initial", &PlanningConfiguration::p_lat_initial)
            .def_readwrite("uncertainty_p_lon", &PlanningConfiguration::uncertainty_p_lon)
            .def_readwrite("uncertainty_p_lat", &PlanningConfiguration::uncertainty_p_lat)
            .def_readwrite("v_lon_initial", &PlanningConfiguration::v_lon_initial)
            .def_readwrite("v_lat_initial", &PlanningConfiguration::v_lat_initial)
            .def_readwrite("uncertainty_v_lon", &PlanningConfiguration::uncertainty_v_lon)
            .def_readwrite("uncertainty_v_lat", &PlanningConfiguration::uncertainty_v_lat)
            .def_readwrite("id_lanelet_initial", &PlanningConfiguration::id_lanelet_initial)
            .def_readwrite("coordinate_system", &PlanningConfiguration::coordinate_system)
            .def_readwrite("reference_point", &PlanningConfiguration::reference_point)
            .def_readwrite("CLCS", &PlanningConfiguration::CLCS);

    py::class_<ReachableSetConfiguration, shared_ptr<ReachableSetConfiguration >>(m, "ReachableSetConfiguration")
            .def(py::init<>())
            .def_readwrite("mode_repartition", &ReachableSetConfiguration::mode_repartition)
            .def_readwrite("mode_inflation", &ReachableSetConfiguration::mode_inflation)
            .def_readwrite("size_grid", &ReachableSetConfiguration::size_grid)
            .def_readwrite("size_grid_2nd", &ReachableSetConfiguration::size_grid_2nd)
            .def_readwrite("radius_terminal_split", &ReachableSetConfiguration::radius_terminal_split)
            .def_readwrite("length_edge_node_min", &ReachableSetConfiguration::length_edge_node_min)
            .def_readwrite("num_threads", &ReachableSetConfiguration::num_threads)
            .def_readwrite("prune_nodes", &ReachableSetConfiguration::prune_nodes)
            .def_readwrite("rasterize_obstacles", &ReachableSetConfiguration::rasterize_obstacles);

    py::class_<DebugConfiguration, shared_ptr<DebugConfiguration >>(m, "DebugConfiguration")
            .def(py::init<>())
            .def_readwrite("verbose_mode", &DebugConfiguration::verbose_mode)
            .def_readwrite("measure_time", &DebugConfiguration::measure_time);
}