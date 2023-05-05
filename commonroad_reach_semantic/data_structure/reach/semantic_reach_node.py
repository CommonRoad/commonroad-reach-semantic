from __future__ import annotations

import copy
from typing import Set

from commonroad_reach.data_structure.reach.reach_node import ReachNode
from commonroad_reach.pycrreach import ReachPolygon

from commonroad_reach_semantic.data_structure.rule.proposition import Proposition as Prop
from commonroad_reach_semantic.data_structure.rule.proposition_holder import PropositionHolder


class SemanticReachNode(ReachNode):
    """Node within the reachability graph.

    Compared to its base class, this one is labeled with a set of propositions.
    """

    def __init__(self, polygon_lon: ReachPolygon, polygon_lat: ReachPolygon,
                 step: int = -1) -> None:
        super().__init__(polygon_lon, polygon_lat, step)

    def clone(self) -> SemanticReachNode:
        """
        Returns a clone of the reach node.
        """
        node_clone = SemanticReachNode(self.polygon_lon.clone(convexify=False),
                                       self.polygon_lat.clone(convexify=False),
                                       self.step)
        node_clone.list_nodes_parent = copy.deepcopy(self.list_nodes_parent)
        node_clone.list_nodes_child = copy.deepcopy(self.list_nodes_child)
        node_clone.source_propagation = self.source_propagation

        return node_clone
