#pragma once

#include "reach_semantic/data_structure/reach/predicates/cpp_predicate.hpp"

using geometry::CurvilinearCoordinateSystem;

namespace semantic_reach {

class KeepSafeDistancePrecPredicate : public CppPredicate {
  private:
    static constexpr int NUM_SUPPORT_POINTS = 2;

    size_t obstacle_id;
    double ego_length;
    double ego_reaction_time;
    double ego_deceleration;
    double vehicle_deceleration;                    //deceleration of other vehicle
    double ego_speed;

        [[nodiscard]] std::vector<reach::ReachNodePtr> _restrict_reach_node_mandatory(
                int step, const reach::ReachNodePtr &reach_node, const std::shared_ptr<World> &world,
                const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ego_ccs) const override;

        [[nodiscard]] std::vector<reach::ReachNodePtr> _restrict_reach_node_forbidden(
                int step, const reach::ReachNodePtr &reach_node, const std::shared_ptr<World> &world,
                const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ego_ccs) const override;


        [[nodiscard]] std::optional<double> _determine_safe_position(
            int step, const reach::ReachNodePtr &reach_node, const std::shared_ptr<World> &world,
            const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ego_ccs, double ego_speed) const;

        [[nodiscard]] double _determine_slope(double ego_speed) const;

        [[nodiscard]] static std::tuple<double, double, double>
        _compute_parameters_of_halfspace(double ego_velocity_support, double slope, double safe_pos);

      public:
        KeepSafeDistancePrecPredicate(bool negated, size_t obstacle_id, double ego_length);

};
}


