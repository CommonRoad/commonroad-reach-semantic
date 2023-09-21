#pragma once

#include "reach_semantic/data_structure/reach/predicates/predicate.hpp"
#include "reach_semantic/data_structure/reach/predicates/predicate_config.hpp"

namespace semantic_reach {
class CppPredicate : public Predicate {
  private:
    bool negated;

    [[nodiscard]] virtual std::vector<reach::ReachNodePtr>
    _restrict_reach_node_mandatory(int step, const reach::ReachNodePtr &reach_node, const std::shared_ptr<World> &world,
                                   const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ego_ccs) const = 0;

    [[nodiscard]] virtual std::vector<reach::ReachNodePtr>
    _restrict_reach_node_forbidden(int step, const reach::ReachNodePtr &reach_node, const std::shared_ptr<World> &world,
                                   const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ego_ccs) const = 0;

  protected:
    std::shared_ptr<PredicateConfiguration> config;

  public:
    explicit CppPredicate(std::shared_ptr<PredicateConfiguration> config, bool negated, bool needs_lanelets);

    [[nodiscard]] std::vector<reach::ReachNodePtr>
    restrict_reach_node(int step, const reach::ReachNodePtr &reach_node,
                        const semantic_reach::SemanticModelPtr &semantic_model, const std::shared_ptr<World> &world,
                        const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ego_ccs) const final;

    [[nodiscard]] std::vector<reach::ReachNodePtr>
    restrict_reach_node(int step, const reach::ReachNodePtr &reach_node,
                        const semantic_reach::SemanticModelPtr &semantic_model, const std::shared_ptr<World> &world,
                        const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ego_ccs,
                        const std::set<int> &node_lanelet_ids) const final;
};
} // namespace semantic_reach
