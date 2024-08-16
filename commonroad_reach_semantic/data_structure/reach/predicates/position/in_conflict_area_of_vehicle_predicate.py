from typing import List, Optional, Set

from commonroad_reach.data_structure.reach.reach_node import ReachNode

import commonroad_reach_semantic.data_structure.reach.predicates.predicate as predicate
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.rule.proposition import Proposition as Prop


class InConflictAreaOfVehiclePredicate(predicate.Predicate):

    def __init__(self, vehicle_id: int, negated: bool):
        super().__init__(negated)
        self.vehicle_id = vehicle_id
        self.needs_lanelets = True

    def to_proposition(self) -> str:
        return Prop.in_conflict_with(self.vehicle_id)

    @predicate.needs_lanelets_set
    def _restrict_reach_node_mandatory(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel,
                                       node_lanelet_ids: Set[int]) -> List[ReachNode]:
        # Retrieve the vehicle object using its ID
        vehicle = semantic_model.vehicle_model.find_vehicle_by_id(self.vehicle_id)

        # Get the lanelet IDs for the ego vehicle and the other vehicle
        lanelets_dir_ego = set(semantic_model.config.planning.route.lanelet_ids)
        lanelets_dir_other = set(vehicle.lanelets_dir)

        # Determine the lanelets that are in the other vehicle's path but not in the ego vehicle's path
        disjoint_other = list(lanelets_dir_other - lanelets_dir_ego)
        if not disjoint_other or node_lanelet_ids.isdisjoint(disjoint_other):
            return []
        else:
            return [reach_node]

    @predicate.needs_lanelets_set
    def _restrict_reach_node_forbidden(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel,
                                       node_lanelet_ids: Set[int]) -> List[ReachNode]:
        # Retrieve the vehicle object using its ID
        vehicle = semantic_model.vehicle_model.find_vehicle_by_id(self.vehicle_id)

        # Get the lanelet IDs for the ego vehicle and the other vehicle
        lanelets_dir_ego = set(semantic_model.config.planning.route.lanelet_ids)
        lanelets_dir_other = set(vehicle.lanelets_dir)

        # Determine the lanelets that are in the other vehicle's path but not in the ego vehicle's path
        disjoint_other = list(lanelets_dir_other - lanelets_dir_ego)
        if not disjoint_other or node_lanelet_ids.isdisjoint(disjoint_other):
            return [reach_node]
        else:
            return []

    # def _get_vehicle_intersecting_lanelet_ids(self, step, semantic_model: SemanticModel) -> Optional[Set[int]]:
    #     if vehicle := semantic_model.vehicle_model.find_vehicle_by_id(self.vehicle_id):
    #         return {
    #             intersecting
    #             for lanelet_id in vehicle.lanelet_ids_at_step(step)
    #             for intersecting in
    #             semantic_model.lanelet_model.dict_id_lanelet_to_set_ids_lanelets_intersecting[lanelet_id]
    #         }
    #     else:
    #         return None
