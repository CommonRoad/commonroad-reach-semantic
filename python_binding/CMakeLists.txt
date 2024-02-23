# create pybind target with source files
pybind11_add_module(pycrreachs MODULE
        pybind.cpp
        pybind_data_structure.cpp
        pybind_utility.cpp
        pybind_reachset.cpp)

# link to external libraries
target_link_libraries(pycrreachs PRIVATE
        crreachs
        pybind11::module)

if (SKBUILD)
    message(STATUS "Installing Python bindings (pycrreachs) via scikit-build-core.")
    install(TARGETS pycrreachs
            LIBRARY DESTINATION ${SKBUILD_PLATLIB_DIR}/commonroad_reach_semantic)
else()
    message(STATUS "Installing Python bindings (pycrreachs) locally: ${CMAKE_CURRENT_LIST_DIR}/../commonroad_reach_semantic")
    set_target_properties(pycrreachs PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/../commonroad_reach_semantic)
endif()

