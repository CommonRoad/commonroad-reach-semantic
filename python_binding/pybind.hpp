#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>

#include "reach_semantic/utility/shared_include.hpp"
#include "reach_semantic/data_structure/semantic_configuration.hpp"
#include "reach_semantic/data_structure/reach/reachable_set_labeler.hpp"
#include "reach_semantic/data_structure/reach/semantic_labeling_reach_set.hpp"
#include "reach_semantic/data_structure/reach/semantic_otf_reach_set.hpp"
#include "reach_semantic/data_structure/semantic_model.hpp"
#include "reach_semantic/data_structure/proposition_holder.hpp"

#include "reachset/utility/collision_checker.hpp"
#include "reachset/utility/shared_using.hpp"

namespace py = pybind11;

void export_data_structures(py::module& m);

void export_utility(py::module& m);

void export_reach(py::module& m);

// ---- export_data_structures()

void export_semantic_model(py::module& m);

void export_traffic_rule_interface(py::module& m);

void export_configuration(py::module& m);

// ---- export_reach()
void export_reachable_set_interface(py::module& m);

void export_reachable_set_labeler(py::module &m);
