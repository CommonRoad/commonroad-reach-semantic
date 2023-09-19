#pragma once

#include "reachset/data_structure/reach/reach_node.hpp"
#include "reach_semantic/data_structure/semantic_model.hpp"

namespace semantic_reach {
    class Predicate {
    private:
        bool negated;

        [[nodiscard]] virtual std::vector<reach::ReachNodePtr>
        _restrict_reach_node_mandatory(int step, const reach::ReachNodePtr &reach_node,
                                       const std::shared_ptr<World> &world,
                                       const std::shared_ptr<CurvilinearCoordinateSystem> &ego_ccs) const = 0;

        [[nodiscard]] virtual std::vector<reach::ReachNodePtr>
        _restrict_reach_node_forbidden(int step, const reach::ReachNodePtr &reach_node,
                                       const std::shared_ptr<World> &world,
                                       const std::shared_ptr<CurvilinearCoordinateSystem> &ego_ccs) const = 0;

    public:
        const bool needs_lanelets;

        explicit Predicate(bool negated, bool needs_lanelets);

        virtual ~Predicate() = default;

        static std::unique_ptr<Predicate> from_proposition(const std::string &proposition, bool is_negated);

        [[nodiscard]] virtual std::vector<reach::ReachNodePtr>
        restrict_reach_node(int step, const reach::ReachNodePtr &reach_node,
                            const semantic_reach::SemanticModelPtr &semantic_model,
                            const std::shared_ptr<World> &world,
                            const std::shared_ptr<CurvilinearCoordinateSystem> &ego_ccs) const;

        [[nodiscard]] virtual std::vector<reach::ReachNodePtr>
        restrict_reach_node(int step, const reach::ReachNodePtr &reach_node,
                            const semantic_reach::SemanticModelPtr &semantic_model,
                            const std::shared_ptr<World> &world,
                            const std::shared_ptr<CurvilinearCoordinateSystem> &ego_ccs,
                            const std::set<int> &node_lanelet_ids) const;
    };
}
