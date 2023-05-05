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

    proposition_holder: PropositionHolder
    set_ids_lanelets: Set[int]

    def __init__(self, polygon_lon: ReachPolygon, polygon_lat: ReachPolygon,
                 step: int = -1, proposition_holder: PropositionHolder = None) -> None:
        super().__init__(polygon_lon, polygon_lat, step)
        self.proposition_holder = proposition_holder if proposition_holder else PropositionHolder()
        self.set_ids_lanelets = set()

    @property
    def set_propositions(self) -> Set[str]:
        return self.proposition_holder.propositions(include_temporary=True)

    @property
    def set_propositions_without_temporary(self) -> Set[str]:
        return self.proposition_holder.propositions(include_temporary=False)

    def collides_with_vehicle(self) -> bool:
        """
        Check if the reachable set collides with a vehicle based on the assigned propositions.
        :return: True if and only if the reachable set collides with another vehicle
        """
        for proposition in self.set_propositions:
            # check if it is aligned with and besides a vehicle
            if Prop.aligned_with() in proposition:
                id_vehicle = int(proposition.split("_")[1][1:])

                if Prop.beside(id_vehicle) in self.set_propositions:
                    return True
        return False

    def clone(self) -> SemanticReachNode:
        """
        Returns a clone of the reach node.
        """
        node_clone = SemanticReachNode(self.polygon_lon.clone(convexify=False),
                                       self.polygon_lat.clone(convexify=False),
                                       self.step, self.proposition_holder.clone())
        node_clone.list_nodes_parent = copy.deepcopy(self.list_nodes_parent)
        node_clone.list_nodes_child = copy.deepcopy(self.list_nodes_child)
        node_clone.source_propagation = self.source_propagation
        node_clone.set_ids_lanelets = self.set_ids_lanelets.copy()

        return node_clone
