#pragma once

#include <commonroad_cpp/world.h>
#include <commonroad_cpp/geometry/curvilinear_coordinate_system.h>

#include "reachset/data_structure/reach/reach_node.hpp"

using geometry::CurvilinearCoordinateSystem;

/**
 * Evaluates whether the kth vehicle is in front of the pth vehicle.
 */
class InFrontOfObstaclePredicate {
private:
    int obstacle_id;
    bool negated;

    [[nodiscard]] std::optional<double>
    _get_obstacle_front(int step, const std::shared_ptr<World> &world,
                        const std::shared_ptr<CurvilinearCoordinateSystem> &ego_ccs) const;

public:
    /**
     * Constructor for in front of obstacle predicate.
     */
    InFrontOfObstaclePredicate(int obstacle_id, bool negated);

    std::vector<reach::ReachNode>
    _restrict_reach_node_mandatory(int step, reach::ReachNode &reach_node, const std::shared_ptr<World> &world,
                                   const std::shared_ptr<CurvilinearCoordinateSystem> &ego_ccs) const;

    std::vector<reach::ReachNode>
    _restrict_reach_node_forbidden(int step, reach::ReachNode &reach_node, const std::shared_ptr<World> &world,
                                   const std::shared_ptr<CurvilinearCoordinateSystem> &ego_ccs) const;
};
