#include "predicate_test_setup.hpp"

#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/roadNetwork/road_network.h>

void TestEnvironments::set_up_environments() {
    size_t id_l1 = 42;
    auto left_border_l1 = std::vector<vertex>{{0, 4}, {10, 4}, {20, 4}, {30, 4}, {40, 4}, {50, 4}};
    auto right_border_l1 = std::vector<vertex>{{0, 0}, {10, 0}, {20, 0}, {30, 0}, {40, 0}, {50, 0}};
    auto type_l1 = std::set<LaneletType>{};
    auto lanelet1 = std::make_shared<Lanelet>(Lanelet{id_l1, left_border_l1, right_border_l1, type_l1});

    auto road_network_one = std::make_shared<RoadNetwork>(std::vector{lanelet1});

    auto state_obs1 = std::make_shared<State>(time_step, 20, 2, 5, 0, 0);
    auto obs1 = std::make_shared<Obstacle>();
    obs1->setId(100);
    obs1->setObstacleRole(ObstacleRole::DYNAMIC);
    obs1->setObstacleType(ObstacleType::car);
    obs1->setCurrentState(state_obs1);
    obs1->setRectangleShape(5, 2);

    world_one = std::make_shared<World>(time_step, road_network_one, std::vector<std::shared_ptr<Obstacle>>{},
                                        std::vector{obs1}, 0.1);

    geometry::EigenPolyline ref_path_one{{0, 2}, {10, 2}, {20, 2}, {30, 2}, {40, 2}, {50, 2}};
    ccs_one = std::make_shared<geometry::CurvilinearCoordinateSystem>(ref_path_one);
}

void TestPredicateConfigs::set_up_configs() {
    config_default = std::make_shared<semantic_reach::PredicateConfiguration>();
}
