# create pybind target with source files
pybind11_add_module(pycrreachs MODULE
        pybind.cpp
        pybind_data_structure.cpp
        pybind_utility.cpp
        pybind_reachset.cpp
        ${REACH_SOURCE})
# add include directories
target_include_directories(pycrreachs PRIVATE
        ${REACH_INCLUDE_DIR})
# link to external libraries
target_link_libraries(pycrreachs PRIVATE
        ${CRDC_LIBRARY}
        ${CRREACH_LIBRARY}
        fcl
        ccd
        ${S11N_LIBRARY}
        yaml-cpp
        pybind11::module)

message(STATUS "Found python bindings for pycrreachs.")

