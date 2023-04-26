#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>

#include "reachset/utility/shared_using.hpp"

#include "reach_semantic/utility/shared_include.hpp"
#include "reachset/utility/collision_checker.hpp"

#include "reach_semantic/data_structure/configuration.hpp"
#include "reach_semantic/data_structure/reach/reach_polygon_boost.hpp"
#include "reach_semantic/data_structure/reach/reach_node.hpp"
#include "reach_semantic/data_structure/reach/reach_set.hpp"
#include "reach_semantic/data_structure/semantic_model.hpp"
#include "reach_semantic/data_structure/proposition_holder.hpp"

namespace py = pybind11;

void export_data_structures(py::module& m);

void export_utility(py::module& m);

void export_reach(py::module& m);

// ---- export_data_structures()

void export_reach_polygon(py::module& m);

void export_reach_node(py::module& m);

void export_semantic_model(py::module& m);

void export_traffic_rule_interface(py::module& m);

void export_configuration(py::module& m);

// ---- export_reach()
void export_reachable_set_interface(py::module& m);
