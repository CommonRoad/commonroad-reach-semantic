# ========= minimum settings
cmake_minimum_required(VERSION 3.20..3.26)

if(SKBUILD)
    set(SEMANTIC_REACH_VERSION ${SKBUILD_PROJECT_VERSION})
else()
    set(SEMANTIC_REACH_VERSION 0.1.0)
endif()

project(reach
        LANGUAGES CXX
        VERSION ${SEMANTIC_REACH_VERSION}
        DESCRIPTION "C++ extension for the commonroad-reach-semantic Python package")

option(ADD_TESTS "Build unit tests" OFF) # flag to add unit tests

# CMP0135 - URL download timestamp
if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
endif()

# make FetchContent_Declare_Fallback available
include(FetchContent)
FetchContent_Declare(
        commonroad_cmake

        GIT_REPOSITORY https://gitlab.lrz.de/tum-cps/commonroad-cmake.git
        GIT_TAG        main
)
FetchContent_MakeAvailable(commonroad_cmake)
list(APPEND CMAKE_MODULE_PATH ${commonroad_cmake_SOURCE_DIR})

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/external) # add path to cmake files for external packages

# ========= general settings
set(CMAKE_CXX_STANDARD 17) # c++ standard
set(CMAKE_CXX_STANDARD_REQUIRED ON) # c++ standard
set(CMAKE_CXX_EXTENSIONS OFF) # turn off compiler extensions

# Necessary for Python modules: never build as shared library
# IMPORTANT: This must be set before finding the necessary packages
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_compile_definitions(BOOST_ALLOW_DEPRECATED_HEADERS) # silence warnings caused by boost using deprecated headers internally

# ========= find necessary packages
find_package(yaml-cpp REQUIRED) # for reading yaml files
find_package(OpenMP REQUIRED) # for parallel computation
if(ADD_TESTS)
    find_package(doctest REQUIRED) # testing with doctest
endif()

include(ExternalPybind)
include(ExternalReach)

# ========= add sub directories
add_subdirectory(cpp) # semantic reachable set
add_subdirectory(python_binding)  # python binding for semantic reachable set
