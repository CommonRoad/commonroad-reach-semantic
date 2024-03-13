set(CRREACHSEM_SRC_FILES
        src/data_structure/reach/reachable_set_labeler.cpp
        src/data_structure/reach/semantic_reach_set.cpp
        src/data_structure/reach/semantic_labeling_reach_set.cpp
        src/data_structure/reach/semantic_otf_reach_set.cpp

        src/data_structure/reach/predicates/predicate.cpp
        src/data_structure/reach/predicates/predicate_config.cpp
        src/data_structure/reach/predicates/predicate_factory.cpp
        src/data_structure/reach/predicates/cpp_predicate.cpp
        src/data_structure/reach/predicates/py_predicate.cpp

        src/data_structure/reach/predicates/position/in_front_of_obstacle_predicate.cpp
        src/data_structure/reach/predicates/position/behind_obstacle_predicate.cpp
        src/data_structure/reach/predicates/position/left_of_obstacle_predicate.cpp
        src/data_structure/reach/predicates/position/right_of_obstacle_predicate.cpp
        src/data_structure/reach/predicates/braking/keeps_safe_distance_prec_predicate.cpp

        src/data_structure/reach/predicates/braking/keeps_safe_distance_prec_predicate.cpp

        src/data_structure/reach/splitters/minterm_reach_node_splitter.cpp
        src/data_structure/reach/splitters/region_reach_node_splitter.cpp

        src/data_structure/model_checking/finite_automaton.cpp

        src/data_structure/environment_model/semantic_model.cpp

        src/data_structure/semantic_configuration.cpp
        src/data_structure/proposition_holder.cpp
        src/data_structure/traffic_rule.cpp
        src/data_structure/position_interval.cpp
        src/data_structure/region.cpp
        src/data_structure/vehicle.cpp

        src/utility/environment_model.cpp
        src/utility/reach_operation.cpp
        src/utility/spot.cpp
)

set(CRREACHSEM_HDR_FILES
        include/reach_semantic/data_structure/reach/reachable_set_labeler.hpp
        include/reach_semantic/data_structure/reach/semantic_reach_set.hpp
        include/reach_semantic/data_structure/reach/semantic_labeling_reach_set.hpp
        include/reach_semantic/data_structure/reach/semantic_otf_reach_set.hpp

        include/reach_semantic/data_structure/reach/predicates/predicate.hpp
        include/reach_semantic/data_structure/reach/predicates/predicate_config.hpp
        include/reach_semantic/data_structure/reach/predicates/predicate_factory.hpp
        include/reach_semantic/data_structure/reach/predicates/cpp_predicate.hpp
        include/reach_semantic/data_structure/reach/predicates/py_predicate.hpp

        include/reach_semantic/data_structure/reach/predicates/position/in_front_of_obstacle_predicate.hpp
        include/reach_semantic/data_structure/reach/predicates/position/behind_obstacle_predicate.hpp
        include/reach_semantic/data_structure/reach/predicates/position/left_of_obstacle_predicate.hpp
        include/reach_semantic/data_structure/reach/predicates/position/right_of_obstacle_predicate.hpp
        include/reach_semantic/data_structure/reach/predicates/braking/keeps_safe_distance_prec_predicate.hpp

        include/reach_semantic/data_structure/reach/splitters/minterm_reach_node_splitter.hpp
        include/reach_semantic/data_structure/reach/splitters/region_reach_node_splitter.hpp

        include/reach_semantic/data_structure/model_checking/finite_automaton.hpp

        include/reach_semantic/data_structure/environment_model/semantic_model.hpp

        include/reach_semantic/data_structure/semantic_configuration.hpp
        include/reach_semantic/data_structure/proposition_holder.hpp
        include/reach_semantic/data_structure/traffic_rule.hpp
        include/reach_semantic/data_structure/position_interval.hpp
        include/reach_semantic/data_structure/region.hpp
        include/reach_semantic/data_structure/vehicle.hpp

        include/reach_semantic/utility/environment_model.hpp
        include/reach_semantic/utility/pybind_includes.hpp
        include/reach_semantic/utility/reach_operation.hpp
        include/reach_semantic/utility/shared_include.hpp
        include/reach_semantic/utility/spot.hpp
)

add_library(crreachsem ${CRREACHSEM_SRC_FILES})

target_sources(crreachsem PRIVATE ${CRREACHSEM_HDR_FILES})

if (CMAKE_VERSION VERSION_GREATER_EQUAL 3.23.0)
    target_sources(crreachsem
            INTERFACE
            FILE_SET crreachsem_headers
            TYPE HEADERS
            BASE_DIRS include
            FILES ${CRREACHSEM_HDR_FILES})
endif ()

# build position independent code so that we can link this in the Python bindings (builds shared lib)
set_property(TARGET crreachsem PROPERTY POSITION_INDEPENDENT_CODE ON)


target_include_directories(crreachsem
        PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/cpp/include>
)

target_link_libraries(crreachsem
        PRIVATE
        spdlog::spdlog
        OpenMP::OpenMP_CXX
)

# Add spot libraries as imported targets, so that we can link against them with CMAKE_LINK_LIBRARIES_ONLY_TARGETS enabled
# TODO: Check if there is a better way to import spot in CMake projects
add_library(spot::spot INTERFACE IMPORTED)
set_property(TARGET spot::spot PROPERTY IMPORTED_LIBNAME "spot")
add_library(spot::bddx INTERFACE IMPORTED)
set_property(TARGET spot::bddx PROPERTY IMPORTED_LIBNAME "bddx")

target_link_libraries(crreachsem
        PUBLIC
        CommonRoadReach::crreach
        Boost::headers
        EnvironmentModel::env_model
        yaml-cpp::yaml-cpp
        spot::spot
        spot::bddx
        pybind11::embed
)

add_library(CommonRoadReachSemantic::crreachsem ALIAS crreachsem)

foreach (target IN ITEMS crreachsem)
    # this adds many warnings to the build. They usually help to find some bugs
    # TODO: Check whether each warning flag is actually supported by the compiler before adding it
    if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wconversion
                -pedantic -Wfatal-errors -Wno-unused-parameter)
    endif ()
    if (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
        target_compile_options(${target} PRIVATE /W3)
    endif ()
endforeach ()
