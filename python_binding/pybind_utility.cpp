#include "pybind.hpp"

namespace py = pybind11;
using namespace reach;

void export_utility(py::module &module) {
    module.def("print_vertices_polygon", &print_vertices_polygon);
    module.def("create_curvilinear_collision_checker", &create_curvilinear_collision_checker);
}