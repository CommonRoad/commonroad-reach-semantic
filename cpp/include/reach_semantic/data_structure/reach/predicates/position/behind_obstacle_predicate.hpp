#pragma once

#include "reach_semantic/data_structure/reach/predicates/cpp_predicate.hpp"

using geometry::CurvilinearCoordinateSystem;

namespace semantic_reach {

class BehindObstaclePredicate : public CppPredicate {
  private:
    size_t obstacle_id;
    double ego_length;

    [[nodiscard]] std::optional<double>
    _get_obstacle_behind(int step, const std::shared_ptr<World> &world,
                         const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ego_ccs) const;

    [[nodiscard]] std::vector<reach::ReachNodePtr> _restrict_reach_node_mandatory(
        int step, const reach::ReachNodePtr &reach_node, const std::shared_ptr<World> &world,
        const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ego_ccs) const override;

    [[nodiscard]] std::vector<reach::ReachNodePtr> _restrict_reach_node_forbidden(
        int step, const reach::ReachNodePtr &reach_node, const std::shared_ptr<World> &world,
        const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ego_ccs) const override;

  public:
    BehindObstaclePredicate(bool negated, size_t obstacle_id, double ego_length);
};
} // namespace semantic_reach