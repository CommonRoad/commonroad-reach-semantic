import re
from abc import ABC, abstractmethod
from typing import List, Optional, Tuple

from commonroad_reach.data_structure.reach.reach_node import ReachNode

from commonroad_reach_semantic.data_structure.environment_model.region import Region
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.rule.proposition import Proposition as Prop


class Predicate(ABC):
    @staticmethod
    def from_proposition(proposition: str):
        if match := re.match(r"InLanelet_(\d+)", proposition):
            return InLaneletPredicate(int(match.group(1)))
        elif match := re.match(r"Behind_V(\d+)", proposition):
            return BehindObstaclePredicate(int(match.group(1)))
        elif match := re.match(r"Beside_V(\d+)", proposition):
            return BesideObstaclePredicate(int(match.group(1)))
        elif match := re.match(r"InFrontOf_V(\d+)", proposition):
            return InFrontOfObstaclePredicate(int(match.group(1)))
        else:
            raise ValueError(f"Unknown proposition: {proposition}")

    @abstractmethod
    def to_proposition(self) -> str:
        pass

    def restrict_reach_node(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel, negated: bool) -> \
            List[ReachNode]:
        if negated:
            restricted_nodes = self.restrict_reach_node_forbidden(step, reach_node, semantic_model)
        else:
            restricted_nodes = self.restrict_reach_node_mandatory(step, reach_node, semantic_model)
        return [
            node for node in restricted_nodes if not node.is_empty
        ]

    @abstractmethod
    def restrict_reach_node_mandatory(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel) -> List[
        ReachNode]:
        pass

    @abstractmethod
    def restrict_reach_node_forbidden(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel) -> List[
        ReachNode]:
        pass


class InLaneletPredicate(Predicate):
    def __init__(self, lanelet_id: int):
        self.lanelet_id = lanelet_id

    def to_proposition(self) -> str:
        return Prop.in_lanelet(self.lanelet_id)

    def restrict_reach_node_mandatory(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel) -> List[
        ReachNode]:
        split_sets = []
        for region in semantic_model.region_model.list_regions:
            if self.lanelet_id in region.set_ids_lanelets:
                if reach_node_new := self._cut_to_region(reach_node, region):
                    split_sets.append(reach_node_new)
        return split_sets

    def restrict_reach_node_forbidden(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel) -> List[
        ReachNode]:
        split_sets = []
        for region in semantic_model.region_model.list_regions:
            if self.lanelet_id not in region.set_ids_lanelets:
                if reach_node_new := self._cut_to_region(reach_node, region):
                    split_sets.append(reach_node_new)
        return split_sets

    @staticmethod
    def _cut_to_region(reach_node: ReachNode, region: Region) -> Optional[ReachNode]:
        if not region.intersects(reach_node.position_rectangle.bounds, coordinate_system="CVLN"):
            return None

        # there is a possibility of intersection
        polygon_intersection = region.polygon_cvln.intersection(reach_node.position_rectangle)

        # empty intersection
        if polygon_intersection.is_empty:
            return None

        # over-approximate by restoring the intersected polygon to axis-aligned rectangle
        bounds_polygon_intersection = polygon_intersection.bounds

        # clone the reach node and cut down to region in the position domain
        reach_node_new = reach_node.clone()
        reach_node_new.intersect_in_position_domain(*bounds_polygon_intersection)
        return reach_node_new


class BehindObstaclePredicate(Predicate):
    def __init__(self, obstacle_id: int):
        self.obstacle_id = obstacle_id

    def to_proposition(self) -> str:
        return Prop.behind(self.obstacle_id)

    def restrict_reach_node_mandatory(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel) -> List[
        ReachNode]:
        if vehicle_rear := self._get_vehicle_rear(step, semantic_model):
            reach_node.intersect_in_position_domain(p_lon_max=vehicle_rear)
            return [reach_node]
        else:
            raise RuntimeError(f"Vehicle with id {self.obstacle_id} not found")

    def restrict_reach_node_forbidden(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel) -> List[
        ReachNode]:
        if vehicle_rear := self._get_vehicle_rear(step, semantic_model):
            reach_node.intersect_in_position_domain(p_lon_min=vehicle_rear)
            return [reach_node]
        else:
            raise RuntimeError(f"Vehicle with id {self.obstacle_id} not found")

    def _get_vehicle_rear(self, step: int, semantic_model: SemanticModel) -> Optional[float]:
        if vehicle := semantic_model.vehicle_model.find_vehicle_by_id(self.obstacle_id):
            return vehicle.p_lon_min_ref(step, semantic_model.config.vehicle.ego.length / 2)
        return None


class BesideObstaclePredicate(Predicate):

    def __init__(self, obstacle_id: int):
        self.obstacle_id = obstacle_id

    def to_proposition(self) -> str:
        return Prop.beside(self.obstacle_id)

    def restrict_reach_node_mandatory(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel) -> List[
        ReachNode]:
        if front_rear := self._get_vehicle_front_rear(step, semantic_model):
            vehicle_front, vehicle_rear = front_rear
            reach_node.intersect_in_position_domain(p_lon_min=vehicle_rear, p_lon_max=vehicle_front)
            return [reach_node]
        else:
            raise RuntimeError(f"Vehicle {self.obstacle_id} not found")

    def restrict_reach_node_forbidden(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel) -> List[
        ReachNode]:
        if front_rear := self._get_vehicle_front_rear(step, semantic_model):
            vehicle_front, vehicle_rear = front_rear
            behind = reach_node.clone()
            behind.intersect_in_position_domain(p_lon_max=vehicle_rear)
            in_front = reach_node  # reuse old reach node
            in_front.intersect_in_position_domain(p_lon_min=vehicle_front)
            return [behind, in_front]
        else:
            raise RuntimeError(f"Vehicle {self.obstacle_id} not found")

    def _get_vehicle_front_rear(self, step: int, semantic_model: SemanticModel) -> Optional[Tuple[float, float]]:
        if vehicle := semantic_model.vehicle_model.find_vehicle_by_id(self.obstacle_id):
            vehicle_front = vehicle.p_lon_max_ref(step, semantic_model.config.vehicle.ego.length / 2)
            vehicle_rear = vehicle.p_lon_min_ref(step, semantic_model.config.vehicle.ego.length / 2)
            return vehicle_front, vehicle_rear
        else:
            return None

class InFrontOfObstaclePredicate(Predicate):

    def __init__(self, obstacle_id: int):
        self.obstacle_id = obstacle_id

    def to_proposition(self) -> str:
        return Prop.in_front_of(self.obstacle_id)

    def restrict_reach_node_mandatory(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel) -> List[
        ReachNode]:
        if vehicle_front := self._get_vehicle_front(step, semantic_model):
            reach_node.intersect_in_position_domain(p_lon_min=vehicle_front)
            return [reach_node]
        else:
            raise RuntimeError(f"Vehicle {self.obstacle_id} not found")

    def restrict_reach_node_forbidden(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel) -> List[
        ReachNode]:
        if vehicle_front := self._get_vehicle_front(step, semantic_model):
            reach_node.intersect_in_position_domain(p_lon_max=vehicle_front)
            return [reach_node]
        else:
            raise RuntimeError(f"Vehicle {self.obstacle_id} not found")

    def _get_vehicle_front(self, step: int, semantic_model: SemanticModel) -> Optional[float]:
        if vehicle := semantic_model.vehicle_model.find_vehicle_by_id(self.obstacle_id):
            return vehicle.p_lon_max_ref(step, semantic_model.config.vehicle.ego.length / 2)
        else:
            return None
