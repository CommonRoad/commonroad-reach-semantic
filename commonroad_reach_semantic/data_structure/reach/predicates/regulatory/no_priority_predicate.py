from typing import List, Set

from commonroad_reach.data_structure.reach.reach_node import ReachNode

import commonroad_reach_semantic.data_structure.reach.predicates.predicate as predicate
from commonroad_reach_semantic.data_structure.config.outgoing_direction import OutgoingDirection
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.rule.proposition import Proposition as Prop


class NoPriorityPredicate(predicate.Predicate):
    def __init__(self, vehicle_id: int, direction_ego: OutgoingDirection, direction_other: OutgoingDirection,
                 negated: bool):
        super().__init__(negated)
        self.vehicle_id = vehicle_id
        self.direction_ego = direction_ego
        self.direction_other = direction_other

        self.needs_lanelets = True

    def to_proposition(self) -> str:
        return Prop.no_priority(self.vehicle_id, self.direction_ego, self.direction_other)

    @predicate.needs_lanelets_set
    def _restrict_reach_node_mandatory(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel,
                                       node_lanelet_ids: Set[int]) -> List[ReachNode]:
        return [reach_node] if self._other_has_priority(semantic_model, node_lanelet_ids, step) else []

    @predicate.needs_lanelets_set
    def _restrict_reach_node_forbidden(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel,
                                       node_lanelet_ids: Set[int]) -> List[ReachNode]:
        return [reach_node] if not self._other_has_priority(semantic_model, node_lanelet_ids, step) else []

    def _other_has_priority(self, semantic_model, node_lanelet_ids, step):
        """Check if the other vehicle has priority over the ego vehicle for the indicated directions at the given step.

        The other vehicle has priority if any of its occupied lanelets has priority over all occupied lanelets of the ego vehicle.
        See "Formalization of Intersection Traffic Rules in Temporal Logic" (Maierhofer et al., 2022)
        """
        if vehicle := semantic_model.vehicle_model.find_vehicle_by_id(self.vehicle_id):
            region = semantic_model.region_model.find_region_by_lanelet_ids(node_lanelet_ids)
            return any(
                all(
                    vehicle.dict_id_lanelet_to_priorities[lanelet_other][self.direction_other] >
                    region.dict_id_lanelet_to_priorities[lanelet_ego][self.direction_ego]
                    for lanelet_ego in node_lanelet_ids
                )
                for lanelet_other in vehicle.lanelet_ids_at_step(step)
            )
        else:
            raise RuntimeError(f"Vehicle {self.vehicle_id} not found")
