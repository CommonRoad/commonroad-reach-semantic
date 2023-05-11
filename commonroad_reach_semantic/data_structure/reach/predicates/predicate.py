from abc import ABC, abstractmethod
from typing import List, Optional

from commonroad_reach.data_structure.reach.reach_node import ReachNode

from commonroad_reach_semantic.data_structure.environment_model.region import Region
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel


class Predicate(ABC):
    negated: bool

    def __init__(self, negated: bool):
        self.negated = negated

    @abstractmethod
    def to_proposition(self) -> str:
        pass

    def restrict_reach_node(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel) -> \
            List[ReachNode]:
        if self.negated:
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
