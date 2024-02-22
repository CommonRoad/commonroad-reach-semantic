#pragma once

#include "reach_semantic/data_structure/reach/predicates/cpp_predicate.hpp"

using geometry::CurvilinearCoordinateSystem;

namespace semantic_reach {

class KeepSafeDistancePrecPredicate : public CppPredicate {
  private:
    static constexpr int NUM_SUPPORT_POINTS = 10;

    size_t obstacle_id;
    double ego_length;
    double ego_reaction_time;
    double ego_deceleration;
    double other_deceleration; // deceleration of other vehicle specified by obstacle_id

    [[nodiscard]] std::vector<reach::ReachNodePtr> _restrict_reach_node_mandatory(
        int step, const reach::ReachNodePtr &reach_node, const std::shared_ptr<World> &world,
        const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ego_ccs) const override;

    [[nodiscard]] std::vector<reach::ReachNodePtr> _restrict_reach_node_forbidden(
        int step, const reach::ReachNodePtr &reach_node, const std::shared_ptr<World> &world,
        const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ego_ccs) const override;

    /**
     * Evaluate the safe position function at the given velocity.
     *
     * @param step Current time step
     * @param world Environment model
     * @param ego_ccs Curvilinear coordinate system of the ego vehicle
     * @param ego_velocity Velocity at which to evaluate the safe position function
     * @return Maximal longitudinal position of the ego vehicle is still safe at the given velocity or nullopt if the
     * safe position cannot be computed due to prediction or projection domain issues.
     */
    [[nodiscard]] std::optional<double>
    _determine_safe_position(int step, const std::shared_ptr<World> &world,
                             const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ego_ccs,
                             double ego_velocity) const;

    /**
     * Determine the slope (derivative) of the safe position function at the given velocity.
     *
     * @param ego_velocity Velocity at which to determine the slope
     * @return Slope of the safe position function at ego_velocity
     */
    [[nodiscard]] double _determine_slope(double ego_velocity) const;

    /**
     * Compute the coefficients of the halfspace that defines the safe region.
     *
     * @param ego_velocity_support Support point for velocity
     * @param slope Slope of the safe position function at the support point
     * @param safe_pos Value of the safe position function at the support point
     * @return Halfspace coefficients (a, b, c) such that a * s + b * v <= c defines the safe region
     */
    [[nodiscard]] static std::tuple<double, double, double>
    _compute_halfspace_coefficients(double ego_velocity_support, double slope, double safe_pos);

  public:
    /**
     * Constructor for safe distance predicate.
     *
     * @param negated Whether the predicate is negated.
     * @param obstacle_id ID of the other vehicle to keep a safe distance to.
     * @param ego_length Length of the ego vehicle.
     * @param ego_reaction_time Reaction time of the ego vehicle.
     * @param ego_deceleration Deceleration of the ego vehicle.
     * @param vehicle_deceleration Deceleration of the other vehicle.
     */
    KeepSafeDistancePrecPredicate(bool negated, size_t obstacle_id, double ego_length, double ego_reaction_time,
                                  double ego_deceleration, double vehicle_deceleration);
};
} // namespace semantic_reach
