#include <gtest/gtest.h>

#include <commonroad_cpp/predicates/predicate_config.h>

int main(int argc, char **argv) {
    // TODO: remove this dirty fix for undefined symbol checkParameterValidity once it is fixed in env model
    PredicateParameters params{};

    testing::InitGoogleTest(&argc, argv);
    // set the gtest death test style to threadsafe
    testing::FLAGS_gtest_death_test_style = "threadsafe";

    return RUN_ALL_TESTS();
}
