# set a variable to hold all relevant include directories
set(REACHS_INCLUDE_DIR
        ${CMAKE_CURRENT_LIST_DIR}/include
        CACHE STRING "" FORCE)

# set a variable to hold all relevant cpp source files of reachable sets
set(REACHS_SOURCE
        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/reach/reachable_set_labeler.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/reach/semantic_reach_set.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/reach/semantic_labeling_reach_set.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/reach/semantic_otf_reach_set.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/reach/predicates/predicate.cpp

        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/reach/splitters/minterm_reach_node_splitter.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/reach/splitters/region_reach_node_splitter.cpp

        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/model_checking/finite_automaton.cpp

        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/semantic_configuration.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/proposition_holder.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/semantic_model.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/traffic_rule.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/position_interval.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/region.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/vehicle.cpp

        ${CMAKE_CURRENT_LIST_DIR}/src/utility/reach_operation.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/utility/spot.cpp

        CACHE STRING "" FORCE)

# ========= building library
# 1. create library from source files above
add_library(crreachs STATIC ${REACHS_SOURCE})
# build position independent code so that we can link this in the Python bindings (builds shared lib)
set_property(TARGET crreachs PROPERTY POSITION_INDEPENDENT_CODE ON)
# handle pybind11 warning "... declared with greater visibility than the type of its field ..."
set_property(TARGET crreachs PROPERTY CXX_VISIBILITY_PRESET hidden)
# 2. include directories
target_include_directories(crreachs PUBLIC
        ${REACHS_INCLUDE_DIR})
# 3. link to external libraries
target_link_libraries(crreachs PUBLIC
        crreach
        yaml-cpp
        spot
        bddx
        OpenMP::OpenMP_CXX)
target_link_libraries(crreachs PRIVATE pybind11::embed)

# ==== unit tests using Doctest
if (ADD_TESTS)
    # 1. create target
    add_executable(crreachs_tests
            tests/utility/test_spot.cpp
    )
    # 2. include directories
    target_include_directories(crreachs_tests PRIVATE
            ${CMAKE_CURRENT_LIST_DIR}/tests/)
    # 3. link to external libraries
    target_link_libraries(crreachs_tests crreachs)
    # 4. tell doctest to use main for implementation
    target_compile_definitions(crreachs_tests PRIVATE DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN)
endif ()
