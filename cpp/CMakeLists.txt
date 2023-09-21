# set include & library paths variables for external commonroad libraries
set(CRDC_INCLUDE
        ${CRDC_DIR}/cpp/collision/include # drivability checker
        ${CRDC_DIR}/cpp/geometry/include) # curvilinear collision checker

set(CRDC_LIBRARY
        ${CRDC_DIR}/commonroad_dc/libcrcc.a # drivability checker
        # FIXME: temporary fix for duplicate CCS class from drivability checker and environment model
        # for now, we do not link against the CCS part of the drivability checker, which fixes the problem
        # change this back once the environment model uses the CCS of the drivability checker again
        # ${CRDC_DIR}/commonroad_dc/libcrccosy.a # curvilinear coordinate system
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

        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/reach/predicates/predicate.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/reach/predicates/cpp_predicate.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/reach/predicates/py_predicate.cpp

        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/reach/predicates/position/in_front_of_obstacle_predicate.cpp

        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/model_checking/finite_automaton.cpp

        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/environment_model/semantic_model.cpp

        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/semantic_configuration.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/proposition_holder.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/traffic_rule.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/position_interval.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/region.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/vehicle.cpp

        ${CMAKE_CURRENT_LIST_DIR}/src/utility/environment_model.cpp
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
        EnvironmentModel::env_model
        fcl
        ccd
        yaml-cpp
        spot
        bddx)
target_link_libraries(crreachs PRIVATE pybind11::embed)

# ==== unit tests using Doctest
if(ADD_TESTS)
    # 1. create target
    add_executable(doctest_runner
            tests/utility/test_spot.cpp
    )
    # 2. include directories
    target_include_directories(doctest_runner PRIVATE
            ${CMAKE_CURRENT_LIST_DIR}/tests/)
    # 3. link to external libraries
    target_link_libraries(doctest_runner crreachs)
    # 4. tell doctest to use main for implementation
    target_compile_definitions(doctest_runner PRIVATE DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN)
endif()
