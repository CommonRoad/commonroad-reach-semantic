#pragma once

#include "reach_semantic/data_structure/reach/predicates/predicate_config.hpp"

#include <commonroad_cpp/geometry/curvilinear_coordinate_system.h>
#include <commonroad_cpp/world.h>

struct TestEnvironments {
    static constexpr size_t time_step = 0;

    static constexpr size_t id_obstacle_one = 100;
    static constexpr size_t id_lanelet_one = 42;

    std::shared_ptr<World> world_one;
    std::shared_ptr<geometry::CurvilinearCoordinateSystem> ccs_one;

    void set_up_environments();
};

struct TestPredicateConfigs {
    std::shared_ptr<semantic_reach::PredicateConfiguration> config_default;

    void set_up_configs();
};
