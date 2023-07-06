# create pybind target with source files
pybind11_add_module(pycrreachs MODULE
        pybind.cpp
        pybind_benchmark.cpp
        pybind_data_structure.cpp
        pybind_utility.cpp
        pybind_reachset.cpp)
# link to external libraries
target_link_libraries(pycrreachs PRIVATE
        crreachs
        pybind11::module)

message(STATUS "Found python bindings for pycrreachs.")

