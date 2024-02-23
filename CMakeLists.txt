# ========= minimum settings
cmake_minimum_required(VERSION 3.20..3.26)

if(SKBUILD)
    set(SEMANTIC_REACH_VERSION ${SKBUILD_PROJECT_VERSION})
else()
    set(SEMANTIC_REACH_VERSION 0.1.0)
endif()

project(semantic_reach
        LANGUAGES CXX
        VERSION ${SEMANTIC_REACH_VERSION}
        DESCRIPTION "C++ extension for the commonroad-reach-semantic Python package")

option(ADD_TESTS "Build unit tests" OFF) # flag to add unit tests

# CMP0135 - URL download timestamp
if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
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
if (CMAKE_VERSION VERSION_GREATER_EQUAL 3.24.0)
    set(CMAKE_SUPPORTS_TRY_FIND_PACKAGE ON)
else ()
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
endif ()

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
if (CRREACHS_SYSTEM_PACKAGES_FORCE)
    # This is the fallback branch in case the CMake version is older than 3.24
    # and the user requested we try to use system packages
    # For CMake > 3.24, fallback is automatic through the FetchContent find_package
    # integration.

    message(WARNING "CRREACHS_SYSTEM_PACKAGES_FORCE is set - trying to satisfy "
            "all dependencies using installed system packages.\n"
            "If this fails, consider disabling CRREACHS_SYSTEM_PACKAGES_FORCE and "
            "trying again.")

    find_package(spdlog 1.8.0 REQUIRED)
    find_package(GTest)
    find_package(yaml-cpp 0.6.0 REQUIRED)

    # yaml-cpp::yaml-cpp is not present in installed config file as of version 0.7.0
    if (NOT TARGET yaml-cpp::yaml-cpp)
        add_library(yaml-cpp::yaml-cpp ALIAS yaml-cpp)
    endif ()
else ()
    # Normal path: We try to use find_package via FetchContent, otherwise we fall
    # back to normal FetchContent

    include(ExternalSpdlog)
    include(ExternalGoogleTest)
    include(ExternalYamlCpp)
endif ()

find_package(OpenMP REQUIRED) # for parallel computation

include(ExternalPybind)
include(ExternalReach)

# FIXME: This is a dirty hack to make the environment model believe that we never build the Python bindings.
# Otherwise, it tries to install its own bindings, when we want to install ours.
set(WAS_SKBUILD ${SKBUILD})
set(SKBUILD OFF)
include(ExternalEnvironmentModel)
set(SKBUILD ${WAS_SKBUILD})

# ========= add sub directories
add_subdirectory(cpp) # semantic reachable set
add_subdirectory(python_binding)  # python binding for semantic reachable set
