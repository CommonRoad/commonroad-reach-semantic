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
        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/reach/predicates/predicate_config.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/reach/predicates/predicate_factory.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/reach/predicates/cpp_predicate.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/reach/predicates/py_predicate.cpp

        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/reach/predicates/position/in_front_of_obstacle_predicate.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/reach/predicates/position/behind_obstacle_predicate.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/reach/predicates/position/left_of_obstacle_predicate.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/reach/predicates/position/right_of_obstacle_predicate.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/reach/predicates/braking/keeps_safe_distance_prec_predicate.cpp

        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/reach/predicates/braking/keeps_safe_distance_prec_predicate.cpp

        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/reach/splitters/minterm_reach_node_splitter.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/data_structure/reach/splitters/region_reach_node_splitter.cpp

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
        EnvironmentModel::env_model
        yaml-cpp
        spot
        bddx
        pybind11::embed
        OpenMP::OpenMP_CXX)
target_link_libraries(crreachs PRIVATE
        spdlog::spdlog)

# ==== unit tests
if (ADD_TESTS)
    add_subdirectory(tests)
endif ()

# dummy target for CLion to detect header files
add_custom_target(crreachs_headers SOURCES
        ${CMAKE_CURRENT_LIST_DIR}/include/reach_semantic/data_structure/reach/reachable_set_labeler.hpp
        ${CMAKE_CURRENT_LIST_DIR}/include/reach_semantic/data_structure/reach/semantic_reach_set.hpp
        ${CMAKE_CURRENT_LIST_DIR}/include/reach_semantic/data_structure/reach/semantic_labeling_reach_set.hpp
        ${CMAKE_CURRENT_LIST_DIR}/include/reach_semantic/data_structure/reach/semantic_otf_reach_set.hpp

        ${CMAKE_CURRENT_LIST_DIR}/include/reach_semantic/data_structure/reach/predicates/predicate.hpp
        ${CMAKE_CURRENT_LIST_DIR}/include/reach_semantic/data_structure/reach/predicates/predicate_config.hpp
        ${CMAKE_CURRENT_LIST_DIR}/include/reach_semantic/data_structure/reach/predicates/predicate_factory.hpp
        ${CMAKE_CURRENT_LIST_DIR}/include/reach_semantic/data_structure/reach/predicates/cpp_predicate.hpp
        ${CMAKE_CURRENT_LIST_DIR}/include/reach_semantic/data_structure/reach/predicates/py_predicate.hpp

        ${CMAKE_CURRENT_LIST_DIR}/include/reach_semantic/data_structure/reach/predicates/position/in_front_of_obstacle_predicate.hpp
        ${CMAKE_CURRENT_LIST_DIR}/include/reach_semantic/data_structure/reach/predicates/position/behind_obstacle_predicate.hpp
        ${CMAKE_CURRENT_LIST_DIR}/include/reach_semantic/data_structure/reach/predicates/position/left_of_obstacle_predicate.hpp
        ${CMAKE_CURRENT_LIST_DIR}/include/reach_semantic/data_structure/reach/predicates/position/right_of_obstacle_predicate.hpp

        ${CMAKE_CURRENT_LIST_DIR}/include/reach_semantic/data_structure/reach/predicates/braking/keeps_safe_distance_prec_predicate.hpp

        ${CMAKE_CURRENT_LIST_DIR}/include/reach_semantic/data_structure/reach/splitters/minterm_reach_node_splitter.hpp
        ${CMAKE_CURRENT_LIST_DIR}/include/reach_semantic/data_structure/reach/splitters/region_reach_node_splitter.hpp

        ${CMAKE_CURRENT_LIST_DIR}/include/reach_semantic/data_structure/model_checking/finite_automaton.hpp

        ${CMAKE_CURRENT_LIST_DIR}/include/reach_semantic/data_structure/environment_model/semantic_model.hpp

        ${CMAKE_CURRENT_LIST_DIR}/include/reach_semantic/data_structure/semantic_configuration.hpp
        ${CMAKE_CURRENT_LIST_DIR}/include/reach_semantic/data_structure/proposition_holder.hpp
        ${CMAKE_CURRENT_LIST_DIR}/include/reach_semantic/data_structure/traffic_rule.hpp
        ${CMAKE_CURRENT_LIST_DIR}/include/reach_semantic/data_structure/position_interval.hpp
        ${CMAKE_CURRENT_LIST_DIR}/include/reach_semantic/data_structure/region.hpp
        ${CMAKE_CURRENT_LIST_DIR}/include/reach_semantic/data_structure/vehicle.hpp

        ${CMAKE_CURRENT_LIST_DIR}/include/reach_semantic/utility/environment_model.hpp
        ${CMAKE_CURRENT_LIST_DIR}/include/reach_semantic/utility/pybind_includes.hpp
        ${CMAKE_CURRENT_LIST_DIR}/include/reach_semantic/utility/reach_operation.hpp
        ${CMAKE_CURRENT_LIST_DIR}/include/reach_semantic/utility/shared_include.hpp
        ${CMAKE_CURRENT_LIST_DIR}/include/reach_semantic/utility/spot.hpp
)
