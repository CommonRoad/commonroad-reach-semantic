set(CRREACHS_TEST_SRC_FILES
        data_structure/reach/predicates/predicate_test_setup.cpp
        data_structure/reach/predicates/position/test_behind_obstacle_predicate.cpp
        data_structure/reach/predicates/position/test_in_front_of_obstacle_predicate.cpp
        data_structure/reach/predicates/position/test_behind_obstacle_predicate.cpp
        data_structure/reach/predicates/position/test_left_of_obstacle_predicate.cpp
        data_structure/reach/predicates/position/test_right_of_obstacle_predicate.cpp
        utility/test_spot.cpp
)

add_executable(crreachs_test
        ${CRREACHS_TEST_SRC_FILES}
        all_tests.cpp
)

target_link_libraries(crreachs_test PRIVATE
        crreachs
        GTest::gtest)

target_precompile_headers(crreachs_test PRIVATE
        <gtest/gtest.h>)

include(GoogleTest)
# queries the compiled executable for tests, this requires the executable to be runnable. if you are
# cross compiling, make sure to properly set CMAKE_CROSSCOMPILING_EMULATOR.
gtest_discover_tests(
        crreachs_test
        # TEST_PREFIX "./"
        # increase the discovery timeout for `crreachs_test --gtest_list_tests`
        DISCOVERY_TIMEOUT 60
        WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
        XML_OUTPUT_DIR ${PROJECT_BINARY_DIR}/test-reports
)
