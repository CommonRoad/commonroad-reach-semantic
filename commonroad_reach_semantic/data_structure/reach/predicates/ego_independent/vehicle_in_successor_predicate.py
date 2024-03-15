from typing import List, Set, Optional

from commonroad_reach.data_structure.reach.reach_node import ReachNode

import commonroad_reach_semantic.data_structure.reach.predicates.predicate as predicate
from commonroad_reach_semantic.data_structure.config.outgoing_direction import OutgoingDirection
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.rule.proposition import Proposition as Prop


class VehicleInSuccessorPredicate(predicate.Predicate):
    """Indicates whether a vehicle is in a successor lanelet (in the given direction) of its incoming lanelet.

    So this predicate is true for the direction STRAIGHT if the vehicle is going straight in the intersection and has already left the intersection.
    Thus, restricting a reach node with this predicate will either completely remove the reach node or leave it unchanged, because the predicate is independet of the reach node.
    """

    def __init__(self, vehicle_id: int, direction: OutgoingDirection, negated: bool):
        super().__init__(negated)
        self.vehicle_id = vehicle_id
        self.outgoing_direction = direction

    def to_proposition(self) -> str:
        return Prop.in_direction_successor(self.outgoing_direction, self.vehicle_id)

    def _restrict_reach_node_mandatory(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel,
                                       node_lanelet_ids: Optional[Set[int]] = None) -> List[ReachNode]:
        return [reach_node] if self._is_vehicle_in_successor(step, semantic_model) else []

    def _restrict_reach_node_forbidden(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel,
                                       node_lanelet_ids: Optional[Set[int]] = None) -> List[ReachNode]:
        return [reach_node] if not self._is_vehicle_in_successor(step, semantic_model) else []

    def _is_vehicle_in_successor(self, step: int, semantic_model: SemanticModel) -> bool:
        if vehicle := semantic_model.vehicle_model.find_vehicle_by_id(self.vehicle_id):
            # If the vehicle's outgoing direction does not match the predicate's outgoing direction,
            # the vehicle can never be in the successor
            if self.outgoing_direction != vehicle.type_outgoing:
                return False
            vehicle_current_lanelets = vehicle.lanelet_ids_at_step(step)
            vehicle_successor_lanelets = vehicle.set_ids_lanelets_successor_incoming
            return not vehicle_successor_lanelets.isdisjoint(vehicle_current_lanelets)
        else:
            raise RuntimeError(f"Vehicle {self.vehicle_id} not found")
