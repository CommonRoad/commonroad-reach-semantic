# set include & library paths variables for external commonroad libraries
set(CRDC_INCLUDE
        ${CRDC_DIR}/cpp/collision/include # drivability checker
        ${CRDC_DIR}/cpp/geometry/include) # curvilinear collision checker

set(CRDC_LIBRARY
        ${CRDC_DIR}/commonroad_dc/libcrcc.a # drivability checker
        ${CRDC_DIR}/commonroad_dc/libcrccosy.a # curvilinear collision checker
        ${CRDC_DIR}/commonroad_dc/libgpc.a # curvilinear collision checker
        CACHE STRING "" FORCE)

set(CRREACH_INCLUDE ${CRREACH_DIR}/cpp/include)
set(CRREACH_LIBRARY ${CRREACH_DIR}/build/temp.linux-x86_64-cpython-${PYTHON_VER}/cpp/libcrreach.a)

# set a variable to hold all relevant include directories
set(REACH_INCLUDE_DIR
        ${CMAKE_CURRENT_LIST_DIR}/include
        ${CRDC_INCLUDE}
        ${CRREACH_INCLUDE}
        ${EIGEN3_INCLUDE_DIR}
        CACHE STRING "" FORCE)

# set a variable to hold all relevant cpp source files of reachable sets
set(REACH_SOURCE
        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/reach/reachable_set_labeler.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/reach/semantic_reach_set.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/reach/semantic_labeling_reach_set.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/reach/semantic_otf_reach_set.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/reach/semantic_splitting_otf_reach_set.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/reach/predicates/predicate.cpp

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
add_library(crreachs STATIC ${REACH_SOURCE})
# build position independent code so that we can link this in the Python bindings (builds shared lib)
set_property(TARGET crreachs PROPERTY POSITION_INDEPENDENT_CODE ON)
# 2. include directories
target_include_directories(crreachs PUBLIC
        ${REACH_INCLUDE_DIR})
# 3. link to external libraries
target_link_libraries(crreachs PUBLIC
        ${CRREACH_LIBRARY}
        ${CRDC_LIBRARY}
        fcl
        ccd
        ${S11N_LIBRARY}
        yaml-cpp
        spot
        bddx)
target_link_libraries(crreachs PRIVATE pybind11::embed)

# ====
# 1. create executables which uses the source files set above
add_executable(playground_semantic
        ${CMAKE_CURRENT_LIST_DIR}/src/playground_semantic.cpp)
# 2. link to external libraries
target_link_libraries(playground_semantic PRIVATE
        crreachs
        pybind11::embed)

## ==== unit tests using Doctest
## 1. create target
#add_executable(doctest_runner
#        ${REACH_SOURCE}
#        ${CMAKE_CURRENT_LIST_DIR}/tests/test_utility.cpp
#        ${CMAKE_CURRENT_LIST_DIR}/tests/data_structure/test_reach_polygon.cpp
#        ${CMAKE_CURRENT_LIST_DIR}/tests/data_structure/test_reach_base_set.cpp
#        ${CMAKE_CURRENT_LIST_DIR}/tests/data_structure/test_reach_node.cpp
#        ${CMAKE_CURRENT_LIST_DIR}/tests/data_structure/test_segment_tree.cpp
#        ${CMAKE_CURRENT_LIST_DIR}/tests/utility/test_reach_operation.cpp
#        ${CMAKE_CURRENT_LIST_DIR}/tests/utility/test_minkowski.cpp
#        ${CMAKE_CURRENT_LIST_DIR}/tests/utility/test_sweep_line.cpp
#        ${CMAKE_CURRENT_LIST_DIR}/tests/utility/test_collision_checker.cpp
#        ${CMAKE_CURRENT_LIST_DIR}/tests/common/test_reachable_set_interface.cpp
#        ${CMAKE_CURRENT_LIST_DIR}/tests/continuous/test_continuous_reachability_analysis.cpp)
## 2. include directories
#target_include_directories(doctest_runner PRIVATE
#        ${REACH_INCLUDE_DIR}
#        ${CMAKE_CURRENT_LIST_DIR}/tests/)
## 3. link to external libraries
#target_link_libraries(doctest_runner
#        yaml-cpp
#        ${CRDC_LIBRARY}
#        ${CRCCOSY_LIBRARY}
#        fcl
#        ccd
#        ${S11N_LIBRARY})