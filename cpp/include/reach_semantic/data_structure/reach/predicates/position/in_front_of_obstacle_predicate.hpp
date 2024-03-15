#pragma once

#include "reach_semantic/data_structure/reach/predicates/cpp_predicate.hpp"

using geometry::CurvilinearCoordinateSystem;

namespace semantic_reach {
/**
 * Predicate that is true if and only if the ego vehicle's rear is in front of the obstacle's front.
 */
class InFrontOfObstaclePredicate : public CppPredicate {
  private:
    size_t obstacle_id;
    double ego_length;

    /**
     * Get the front of the obstacle at the given step.
     *
     * @param step The considered step.
     * @param world Model of the environment.
     * @param ego_ccs Curvilinear coordinate system of the ego vehicle.
     * @return Front position inflated by half the length of the ego vehicle, or nullopt if the obstacle has no state at
     * the given step or is not in the projection domain of the ego_ccs.
     * @throws std::logic_error If the obstacle does not exist in the world.
     */
    [[nodiscard]] std::optional<double>
    _get_obstacle_front(int step, const std::shared_ptr<World> &world,
                        const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ego_ccs) const;

    [[nodiscard]] std::vector<reach::ReachNodePtr> _restrict_reach_node_mandatory(
        int step, const reach::ReachNodePtr &reach_node, const std::shared_ptr<World> &world,
        const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ego_ccs) const override;

    [[nodiscard]] std::vector<reach::ReachNodePtr> _restrict_reach_node_forbidden(
        int step, const reach::ReachNodePtr &reach_node, const std::shared_ptr<World> &world,
        const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ego_ccs) const override;

  public:
    /**
     * Constructor for in front of obstacle predicate.
     *
     * @param negated Whether the predicate is negated.
     * @param obstacle_id ID of the obstacle.
     * @param ego_length Length of the ego vehicle.
     */
    InFrontOfObstaclePredicate(bool negated, size_t obstacle_id, double ego_length);
};
} // namespace semantic_reach