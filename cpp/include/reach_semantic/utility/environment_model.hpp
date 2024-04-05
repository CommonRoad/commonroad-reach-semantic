#pragma once

#include "commonroad_cpp/obstacle/obstacle.h"

namespace semantic_reach {
/**
 * Resample the time steps of the obstacles' states to the new time step size.
 *
 * @param obstacles Obstacles to consider
 * @param obstacle_dt Original time step size of the obstacle prediction
 * @param new_dt New time step size
 * @throws std::logic_error if the new time step size is not a multiple of the original time step size
 */
void resample_obstacle_states(const std::vector<std::shared_ptr<Obstacle>> &obstacles, double obstacle_dt,
                              double new_dt);
} // namespace semantic_reach
