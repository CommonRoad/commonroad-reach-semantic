from __future__ import annotations

from typing import Set, List, Union

import numpy as np
from commonroad_reach import pycrreach
from commonroad_reach.data_structure.reach.reach_node import ReachNode


class KripkeNode:
    """
    Class representing a node in a Kripke structure.
    """
    cnt_id = 0

    # todo: change set_propositions to proposition holder?
    def __init__(self, step: int, set_propositions=None, set_nodes_reach: Set[ReachNode] = None):
        self.id = KripkeNode.cnt_id
        KripkeNode.cnt_id += 1

        self.step = step
        self.set_propositions = set_propositions.copy() if set_propositions else None
        # reach-nodes-related
        self.set_nodes_reach: Set[ReachNode] = set()
        self.set_nodes_reach_parent: Set[ReachNode] = set()
        self.set_nodes_reach_child: Set[ReachNode] = set()
        self.set_ids_nodes_reach: Set[int] = set()
        self.set_ids_nodes_reach_parent: Set[int] = set()
        self.set_ids_nodes_reach_child: Set[int] = set()

        # kripke-nodes-related
        self.list_nodes_kripke_parent: List[KripkeNode] = list()
        self.list_nodes_kripke_child: List[KripkeNode] = list()
        self.list_ids_nodes_kripke_parent: List[int] = list()
        self.list_ids_nodes_kripke_child: List[int] = list()

        if set_nodes_reach:
            self.add_reach_nodes(set_nodes_reach)

        # Spot twa state index
        self.idx_state_twa = None
        # Spot proposition formula
        self.formula_proposition = None
        # flag indicating whether this is a dead node
        self.dead = False

    def __repr__(self):
        return f"KripkeNode(step={self.step},id={self.id},#reach_nodes={len(self.set_nodes_reach)})"

    def __key(self):
        return self.id, self.step

    def __hash__(self):
        return hash(self.__key())

    def clone(self) -> KripkeNode:
        node_kripke_cloned = KripkeNode(self.step, self.set_propositions)

        # reach-nodes-related
        node_kripke_cloned.set_nodes_reach = self.set_nodes_reach.copy()
        node_kripke_cloned.set_nodes_reach_parent = self.set_nodes_reach_parent.copy()
        node_kripke_cloned.set_nodes_reach_child = self.set_nodes_reach_child.copy()
        node_kripke_cloned.set_ids_nodes_reach = self.set_ids_nodes_reach.copy()
        node_kripke_cloned.set_ids_nodes_reach_parent = self.set_ids_nodes_reach_parent.copy()
        node_kripke_cloned.set_ids_nodes_reach_child = self.set_ids_nodes_reach_child.copy()

        # kripke-nodes-related
        node_kripke_cloned.list_nodes_kripke_parent = self.list_nodes_kripke_parent.copy()
        node_kripke_cloned.list_nodes_kripke_child = self.list_nodes_kripke_child.copy()
        node_kripke_cloned.list_ids_nodes_kripke_parent = self.list_ids_nodes_kripke_parent.copy()
        node_kripke_cloned.list_ids_nodes_kripke_child = self.list_ids_nodes_kripke_child.copy()

        node_kripke_cloned.idx_state_twa = self.idx_state_twa
        node_kripke_cloned.formula_proposition = self.formula_proposition
        node_kripke_cloned.dead = self.dead

        return node_kripke_cloned

    @property
    def area(self):
        """
        Returns the area of the kripke node based on its contained reach nodes.
        """
        sum_area_nodes_reach = 0

        for node_reach in self.set_nodes_reach:
            sum_area_nodes_reach += (node_reach.p_lon_max - node_reach.p_lon_min) * \
                                    (node_reach.p_lat_max - node_reach.p_lat_min)

        return sum_area_nodes_reach

    @property
    def p_lon_min(self):
        """
        Minimum position of the reach nodes in the longitudinal direction.
        """
        p_lon_min = np.infty

        for node_reach in self.set_nodes_reach:
            p_lon_min = min(p_lon_min, node_reach.p_lon_min)

        return p_lon_min

    @property
    def p_lon_max(self):
        """
        Maximum position of the reach nodes in the longitudinal direction.
        """
        p_lon_max = -np.infty

        for node_reach in self.set_nodes_reach:
            p_lon_max = max(p_lon_max, node_reach.p_lon_max)

        return p_lon_max

    @property
    def p_lat_min(self):
        """
        Minimum position of the reach nodes in the lateral direction.
        """
        p_lat_min = np.infty

        for node_reach in self.set_nodes_reach:
            p_lat_min = min(p_lat_min, node_reach.p_lat_min)

        return p_lat_min

    @property
    def p_lat_max(self):
        """
        Maximum position of the reach nodes in the lateral direction.
        """
        p_lat_max = -np.infty

        for node_reach in self.set_nodes_reach:
            p_lat_max = max(p_lat_max, node_reach.p_lat_max)

        return p_lat_max

    @property
    def v_lon_max(self):
        """
        Maximum velocity of the reach nodes in the longitudinal direction.
        """
        v_lon_max = -np.infty

        for node_reach in self.set_nodes_reach:
            v_lon_max = max(v_lon_max, node_reach.v_lon_max)

        return v_lon_max

    @property
    def v_lon_min(self):
        """
        Minimum velocity of the reach nodes in the longitudinal direction.
        """
        v_lon_min = np.infty

        for node_reach in self.set_nodes_reach:
            v_lon_min = min(v_lon_min, node_reach.v_lon_min)

        return v_lon_min

    def add_reach_node(self, node_reach: Union[ReachNode, pycrreach.ReachNode]):
        """
        Adds a reach node.
        """
        self.set_nodes_reach.add(node_reach)
        self.set_ids_nodes_reach.add(node_reach.id)

        self.set_nodes_reach_parent.update(set(node_reach.list_nodes_parent))
        self.set_ids_nodes_reach_parent.update({node_parent.id for node_parent in node_reach.list_nodes_parent})
        self.set_nodes_reach_child.update(set(node_reach.list_nodes_child))
        self.set_ids_nodes_reach_child.update({node_child.id for node_child in node_reach.list_nodes_child})

    def add_reach_nodes(self, nodes_reach):
        """
        Adds a list/set of reach nodes.
        """
        for node_reach in nodes_reach:
            self.add_reach_node(node_reach)

    def remove_reach_node(self, node_reach: ReachNode):
        """
        Removes a reach node.
        """
        self.set_nodes_reach.discard(node_reach)
        self.set_ids_nodes_reach.discard(node_reach.id)

        set_nodes_parent_intersection = self.set_nodes_reach_parent.intersection(set(node_reach.list_nodes_parent))
        set_nodes_parent_difference = self.set_nodes_reach_parent.difference(set(node_reach.list_nodes_parent))
        self.set_nodes_reach_parent = set_nodes_parent_difference.union(set_nodes_parent_intersection)

        set_nodes_child_intersection = self.set_nodes_reach_child.intersection(set(node_reach.list_nodes_child))
        set_nodes_child_difference = self.set_nodes_reach_child.difference(set(node_reach.list_nodes_child))
        self.set_nodes_reach_child = set_nodes_child_difference.union(set_nodes_child_intersection)

    def remove_reach_nodes(self, nodes_reach):
        """
        Removes a list/set of reach nodes.
        """
        for node_reach in nodes_reach:
            self.remove_reach_node(node_reach)

    def connect_to_parent_kripke_node(self, node_kripke_parent: KripkeNode):
        """
        Establishes a connection between self and the parent Kripke node.
        """
        self.add_parent_kripke_node(node_kripke_parent)
        node_kripke_parent.add_child_kripke_node(self)

    def connect_to_parent_kripke_nodes(self, set_nodes_kripke_parent: Set[KripkeNode]):
        """
        Establishes connections between self and parent Kripke nodes.

        Only operates on the Kripke structure level (not on the reachable set level).
        """
        for node_kripke_parent in set_nodes_kripke_parent:
            self.connect_to_parent_kripke_node(node_kripke_parent)

    def connect_to_child_kripke_node(self, node_kripke_child: KripkeNode):
        """
        Establishes a connection between self and the child Kripke node.
        """
        self.add_child_kripke_node(node_kripke_child)
        node_kripke_child.add_parent_kripke_node(self)

    def connect_to_child_kripke_nodes(self, set_nodes_kripke_child: Set[KripkeNode]):
        """
        Establishes connections between self and child Kripke nodes.

        Only operates on the Kripke structure level (not on the reachable set level).
        """
        for node_kripke_child in set_nodes_kripke_child:
            self.connect_to_child_kripke_node(node_kripke_child)

    def add_parent_kripke_node(self, node_parent: KripkeNode):
        if node_parent not in self.list_nodes_kripke_parent:
            self.list_nodes_kripke_parent.append(node_parent)
            self.list_ids_nodes_kripke_parent.append(node_parent.id)

    def add_child_kripke_node(self, node_child: KripkeNode):
        if node_child not in self.list_nodes_kripke_child:
            self.list_nodes_kripke_child.append(node_child)
            self.list_ids_nodes_kripke_child.append(node_child.id)

    def remove_parent_kripke_node(self, node_parent: KripkeNode):
        if node_parent in self.list_nodes_kripke_parent:
            self.list_nodes_kripke_parent.remove(node_parent)
            self.list_ids_nodes_kripke_parent.remove(node_parent.id)

    def remove_child_kripke_node(self, node_child: KripkeNode):
        if node_child in self.list_nodes_kripke_child:
            self.list_nodes_kripke_child.remove(node_child)
            self.list_ids_nodes_kripke_child.remove(node_child.id)
