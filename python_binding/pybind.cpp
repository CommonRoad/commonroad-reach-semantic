#include "pybind.hpp"

#include <commonroad_cpp/predicates/predicate_config.h>

PYBIND11_MODULE(pycrreachs, m) {
    m.doc() = "Pybind module for semantic reachable set.";

    export_data_structures(m);
    export_utility(m);
    export_reach(m);

    // TODO: remove this dirty fix for undefined symbol checkParameterValidity once it is fixed in env model
    PredicateParameters p{};
}