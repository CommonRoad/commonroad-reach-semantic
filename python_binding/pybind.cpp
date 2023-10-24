#include "pybind.hpp"

#include <commonroad_cpp/predicates/predicate_config.h>

PYBIND11_MODULE(pycrreachs, module) {
    module.doc() = "Pybind module for semantic reachable set.";

    export_data_structures(module);
    export_utility(module);
    export_reach(module);

    // TODO: remove this dirty fix for undefined symbol checkParameterValidity once it is fixed in env model
    PredicateParameters params{};
}