#pragma once

#include <commonroad_cpp/world.h>
#include <commonroad_cpp/geometry/curvilinear_coordinate_system.h>

#include "reach_semantic/data_structure/reach/predicates/predicate.hpp"

using geometry::CurvilinearCoordinateSystem;

 namespace semantic_reach {
     /**
      * Evaluates whether the kth vehicle is in front of the pth vehicle.
      */
     class InFrontOfObstaclePredicate : public Predicate {
     private:
         int obstacle_id;

         [[nodiscard]] std::optional<double>
         _get_obstacle_front(int step, const std::shared_ptr<World> &world,
                             const std::shared_ptr<CurvilinearCoordinateSystem> &ego_ccs) const;

         [[nodiscard]] std::vector<reach::ReachNodePtr>
         _restrict_reach_node_mandatory(int step, const reach::ReachNodePtr &reach_node,
                                        const std::shared_ptr<World> &world,
                                        const std::shared_ptr<CurvilinearCoordinateSystem> &ego_ccs) const override;

         [[nodiscard]] std::vector<reach::ReachNodePtr>
         _restrict_reach_node_forbidden(int step, const reach::ReachNodePtr &reach_node,
                                        const std::shared_ptr<World> &world,
                                        const std::shared_ptr<CurvilinearCoordinateSystem> &ego_ccs) const override;

     public:
         /**
          * Constructor for in front of obstacle predicate.
          */
         InFrontOfObstaclePredicate(int obstacle_id, bool negated);

         static std::optional<std::unique_ptr<InFrontOfObstaclePredicate>>
         try_from_proposition(const std::string &proposition, bool is_negated);
     };
 }