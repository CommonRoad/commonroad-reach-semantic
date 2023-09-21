# ========= minimum settings
cmake_minimum_required(VERSION 3.20..3.25) # minimum required version of CMake
project(semantic_reach) # project name
option(ADD_TESTS "unit tests" OFF) # flag to add unit tests

# CMP0135 - URL download timestamp
if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
endif()

# ========= check for required variables
if(NOT DEFINED PYTHON_VER)
    message(FATAL_ERROR "Please specify the python version via -DPYTHON_VER=")
endif()

if(NOT DEFINED CRDC_DIR) # path to drivability checker
    message(FATAL_ERROR "Please specify the path to drivability checker via -DCRDC_DIR=")
endif()

if(NOT DEFINED CRREACH_DIR) # path to regular commonroad reach
    message(FATAL_ERROR "Please specify the path to regular commonroad reach via -DCRREACH_DIR=")
endif()

include(CMakeDependentOption)

option(CRREACHS_SYSTEM_PACKAGES "Try to use system packages for dependencies" ON)
cmake_dependent_option(CRREACHS_SYSTEM_PACKAGES_FORCE
        "For CMake<3.24: Force using system packages for all dependencies"
        OFF
        "NOT CMAKE_SUPPORTS_TRY_FIND_PACKAGE"
        OFF
)

set(CMAKE_SUPPORTS_TRY_FIND_PACKAGE OFF)
if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.24.0)
    set(CMAKE_SUPPORTS_TRY_FIND_PACKAGE ON)
else()
    message(WARNING "Your CMake version (${CMAKE_VERSION}) does not support "
            "the FetchContent find_package integration introduced in CMake 3.24. "
            "As a fallback, we will simply build all dependencies ourselves "
            "irrespective of whether a suitable system version exists. "
            "While this does not impair functionality, it might slow down the build "
            "process a bit.\n"
            "In case you have all required dependencies installed, you can try "
            "enabling the option\n"
            "\tCRREACHS_SYSTEM_PACKAGES_FORCE\n"
            "which will force using find_package for all dependencies.")
endif()

option(CRREACHS_SYSTEM_PACKAGES "Try to use system packages for dependencies" ON)
cmake_dependent_option(CRREACHS_SYSTEM_PACKAGES_FORCE
        "For CMake<3.24: Force using system packages for all dependencies"
        OFF
        "NOT CMAKE_SUPPORTS_TRY_FIND_PACKAGE"
        OFF
)

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

include(ExternalEnvironmentModel)

# Include Eigen3, spdlog, and yaml-cpp
if(CRREACHS_SYSTEM_PACKAGES_FORCE)
    # This is the fallback branch in case the CMake version is older than 3.24
    # and the user requested we try to use system packages
    # For CMake > 3.24, fallback is automatic through the FetchContent find_package
    # integration.

    message(WARNING "CRREACHS_SYSTEM_PACKAGES_FORCE is set - trying to satisfy "
            "all dependencies using installed system packages.\n"
            "If this fails, consider disabling CRREACHS_SYSTEM_PACKAGES_FORCE and "
            "trying again.")

    find_package(spdlog 1.8.0 REQUIRED)
else()
    # Normal path: We try to use find_package via FetchContent, otherwise we fall
    # back to normal FetchContent

    include(ExternalSpdlog)
endif()

# ========= set variables
# path to FCL installation in drivability checker
set(fcl_DIR ${CRDC_DIR}/build/temp.linux-x86_64-cpython-${PYTHON_VER}/dist/lib/cmake/fcl)
# path to CCD installation in drivability checker
set(ccd_DIR ${CRDC_DIR}/build/temp.linux-x86_64-cpython-${PYTHON_VER}/dist/lib/ccd)

# ========= find necessary packages
find_package(Eigen3 REQUIRED) # Eigen3
find_package(ccd REQUIRED) # for collision checks
find_package(fcl REQUIRED) # for collision checks
find_package(yaml-cpp REQUIRED) # for reading yaml files
find_package(OpenMP REQUIRED) # for parallel computation
if(ADD_TESTS)
    find_package(doctest REQUIRED) # testing with doctest
endif()

# ========= general settings
set(CMAKE_CXX_STANDARD 17) # c++ standard
set(CMAKE_CXX_STANDARD_REQUIRED ON) # c++ standard
set(CMAKE_CXX_EXTENSIONS OFF) # turn off compiler extensions
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}") # enable parallel computation
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}") # enable parallel computation
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}") # enable parallel computation
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0") # for profiling
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0") # for profiling
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../) # output directory of built libraries

set(CMAKE_CXX_VISIBILITY_PRESET hidden) # required by pybind11
add_compile_definitions(BOOST_ALLOW_DEPRECATED_HEADERS) # silence warnings caused by boost using deprecated headers internally

# ========= add sub directories
add_subdirectory(external/pybind11) # pybind header
add_subdirectory(cpp) # reachable set
add_subdirectory(python_binding)  # python binding for reachable set
