from abc import ABC, abstractmethod
from typing import List, Optional, Set, Callable

from commonroad_reach.data_structure.reach.reach_node import ReachNode

from commonroad_reach_semantic.data_structure.environment_model.region import Region
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel


class Predicate(ABC):
    """Abstract class for predicates.

    :ivar negated: True iff the predicate is negated
    :ivar needs_lanelets: True iff the predicate needs information about lanelets to restrict a reach node
    """

    negated: bool
    needs_lanelets: bool

    def __init__(self, negated: bool):
        self.negated = negated
        self.needs_lanelets = False

    @abstractmethod
    def to_proposition(self) -> str:
        """Returns the proposition corresponding to the predicate."""
        pass

    def restrict_reach_node(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel,
                            node_lanelet_ids: Optional[Set[int]] = None) -> List[ReachNode]:
        """Restrict the reach node according to the predicate.

        :param step: The step for which the reach node was calculated
        :param reach_node: The reach node to be restricted (might be modified)
        :param semantic_model: The environment model against which the predicate is evaluated
        :param node_lanelet_ids: The set of lanelet ids that the reach node is in (only required if needs_lanelets is True)
        :returns: A list of restricted reach nodes (this is a list, because restricting might require splitting the reach node)
        """
        if self.negated:
            restricted_nodes = self._restrict_reach_node_forbidden(step, reach_node, semantic_model, node_lanelet_ids)
        else:
            restricted_nodes = self._restrict_reach_node_mandatory(step, reach_node, semantic_model, node_lanelet_ids)
        return [
            node for node in restricted_nodes if not node.is_empty
        ]

    @abstractmethod
    def _restrict_reach_node_mandatory(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel,
                                       node_lanelet_ids: Optional[Set[int]] = None) -> List[ReachNode]:
        """Restrict the reach node according to the predicate, assuming that the predicate is not negated."""
        pass

    @abstractmethod
    def _restrict_reach_node_forbidden(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel,
                                       node_lanelet_ids: Optional[Set[int]] = None) -> List[ReachNode]:
        """Restrict the reach node according to the predicate, assuming that the predicate is negated."""
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


def needs_lanelets_set(func: Callable[[Predicate, int, ReachNode, SemanticModel, Set[int]], List[ReachNode]]) -> \
        Callable[[Predicate, int, ReachNode, SemanticModel, Optional[Set[int]]], List[ReachNode]]:
    """Decorator for predicates that need information about lanelets to restrict a reach node."""

    def check_node_lanelet_ids(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel,
                               node_lanelet_ids: Optional[Set[int]] = None) -> List[ReachNode]:
        if node_lanelet_ids is None:
            raise ValueError(f"{self.__class__.__name__} requires node_lanelet_ids to be set.")
        return func(self, step, reach_node, semantic_model, node_lanelet_ids)

    return check_node_lanelet_ids
