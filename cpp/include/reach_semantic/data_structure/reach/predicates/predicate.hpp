#pragma once

#include "reachset/data_structure/reach/reach_node.hpp"
#include "reach_semantic/data_structure/environment_model/semantic_model.hpp"

#include <commonroad_cpp/world.h>
#include <commonroad_cpp/geometry/curvilinear_coordinate_system.h>

namespace semantic_reach {
    class Predicate {
    public:
        const bool needs_lanelets;

        explicit Predicate(bool needs_lanelets);

        virtual ~Predicate() = default;

        static std::unique_ptr<Predicate> from_proposition(const std::string &proposition, bool is_negated);

        [[nodiscard]] virtual std::vector<reach::ReachNodePtr>
        restrict_reach_node(int step, const reach::ReachNodePtr &reach_node,
                            const semantic_reach::SemanticModelPtr &semantic_model,
                            const std::shared_ptr<World> &world,
                            const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ego_ccs) const = 0;

        [[nodiscard]] virtual std::vector<reach::ReachNodePtr>
        restrict_reach_node(int step, const reach::ReachNodePtr &reach_node,
                            const semantic_reach::SemanticModelPtr &semantic_model,
                            const std::shared_ptr<World> &world,
                            const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ego_ccs,
                            const std::set<int> &node_lanelet_ids) const = 0;
    };
}
