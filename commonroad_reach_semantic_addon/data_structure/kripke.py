import logging
from collections import defaultdict
from typing import List, Set, Optional, Dict, Union

import numpy as np
import spot
import buddy
from commonroad_reach_semantic_addon import pycrreachs
from commonroad_reach_semantic_addon.data_structure.reach.reach_interface import ReachableSetInterface
from commonroad_reach_semantic_addon.data_structure.reach.reach_node import ReachNode
from commonroad_reach_semantic_addon.data_structure.proposition_holder import PropositionHolder
from commonroad_reach_semantic_addon.utility import reach_operation as util_reach
from commonroad_reach_semantic_addon.utility import spot as util_spot
from commonroad_reach_semantic_addon.utility import logger as util_logger

logger = logging.getLogger(__name__)


class KripkeNode:
    """
    Class representing a node in a Kripke structure.
    """
    cnt_id = 0
    backend = None

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
        self.list_nodes_kripke_parent: List['KripkeNode'] = list()
        self.list_nodes_kripke_child: List['KripkeNode'] = list()
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

    def clone(self) -> 'KripkeNode':
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

        if self.backend == "PYTHON":
            for node_reach in self.set_nodes_reach:
                sum_area_nodes_reach += (node_reach.p_lon_max - node_reach.p_lon_min) * \
                                        (node_reach.p_lat_max - node_reach.p_lat_min)

        else:
            for node_reach in self.set_nodes_reach:
                sum_area_nodes_reach += (node_reach.p_lon_max() - node_reach.p_lon_min()) * \
                                        (node_reach.p_lat_max() - node_reach.p_lat_min())

        return sum_area_nodes_reach

    @property
    def p_lon_min(self):
        """
        Minimum position of the reach nodes in the longitudinal direction.
        """
        p_lon_min = np.infty

        if self.backend == "PYTHON":
            for node_reach in self.set_nodes_reach:
                p_lon_min = min(p_lon_min, node_reach.p_lon_min)

        else:
            for node_reach in self.set_nodes_reach:
                p_lon_min = min(p_lon_min, node_reach.p_lon_min())

        return p_lon_min

    @property
    def p_lon_max(self):
        """
        Maximum position of the reach nodes in the longitudinal direction.
        """
        p_lon_max = -np.infty

        if self.backend == "PYTHON":
            for node_reach in self.set_nodes_reach:
                p_lon_max = max(p_lon_max, node_reach.p_lon_max)

        else:
            for node_reach in self.set_nodes_reach:
                p_lon_max = max(p_lon_max, node_reach.p_lon_max())

        return p_lon_max

    @property
    def p_lat_min(self):
        """
        Minimum position of the reach nodes in the lateral direction.
        """
        p_lat_min = np.infty

        if self.backend == "PYTHON":
            for node_reach in self.set_nodes_reach:
                p_lat_min = min(p_lat_min, node_reach.p_lat_min)

        else:
            for node_reach in self.set_nodes_reach:
                p_lat_min = min(p_lat_min, node_reach.p_lat_min())

        return p_lat_min

    @property
    def p_lat_max(self):
        """
        Maximum position of the reach nodes in the lateral direction.
        """
        p_lat_max = -np.infty

        if self.backend == "PYTHON":
            for node_reach in self.set_nodes_reach:
                p_lat_max = max(p_lat_max, node_reach.p_lat_max)

        else:
            for node_reach in self.set_nodes_reach:
                p_lat_max = max(p_lat_max, node_reach.p_lat_max())

        return p_lat_max

    @property
    def v_lon_max(self):
        """
        Maximum velocity of the reach nodes in the longitudinal direction.
        """
        v_lon_max = -np.infty

        if self.backend == "PYTHON":
            for node_reach in self.set_nodes_reach:
                v_lon_max = max(v_lon_max, node_reach.v_lon_max)

        else:
            for node_reach in self.set_nodes_reach:
                v_lon_max = max(v_lon_max, node_reach.v_lon_max())

        return v_lon_max

    @property
    def v_lon_min(self):
        """
        Minimum velocity of the reach nodes in the longitudinal direction.
        """
        v_lon_min = np.infty

        if self.backend == "PYTHON":
            for node_reach in self.set_nodes_reach:
                v_lon_min = min(v_lon_min, node_reach.v_lon_min)

        else:
            for node_reach in self.set_nodes_reach:
                v_lon_min = min(v_lon_min, node_reach.v_lon_min())

        return v_lon_min

    def add_reach_node(self, node_reach: Union[ReachNode, pycrreachs.ReachNode]):
        """
        Adds a reach node.
        """
        self.set_nodes_reach.add(node_reach)
        self.set_ids_nodes_reach.add(node_reach.id)

        if self.backend == "PYTHON":
            parents = "list_nodes_parent"
            children = "list_nodes_child"

        else:
            parents = "vec_nodes_parent"
            children = "vec_nodes_child"

        self.set_nodes_reach_parent.update(set(eval(f"node_reach.{parents}")))
        self.set_ids_nodes_reach_parent.update({node_parent.id for node_parent in eval(f"node_reach.{parents}")})
        self.set_nodes_reach_child.update(set(eval(f"node_reach.{children}")))
        self.set_ids_nodes_reach_child.update({node_child.id for node_child in eval(f"node_reach.{children}")})

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

        if self.backend == "PYTHON":
            parents = "list_nodes_parent"
            children = "list_nodes_child"

        else:
            parents = "vec_nodes_parent"
            children = "vec_nodes_child"

        set_nodes_parent_intersection = self.set_nodes_reach_parent.intersection(set(eval(f"node_reach.{parents}")))
        set_nodes_parent_difference = self.set_nodes_reach_parent.difference(set(eval(f"node_reach.{parents}")))
        self.set_nodes_reach_parent = set_nodes_parent_difference.union(set_nodes_parent_intersection)

        set_nodes_child_intersection = self.set_nodes_reach_child.intersection(set(eval(f"node_reach.{children}")))
        set_nodes_child_difference = self.set_nodes_reach_child.difference(set(eval(f"node_reach.{children}")))
        self.set_nodes_reach_child = set_nodes_child_difference.union(set_nodes_child_intersection)

    def remove_reach_nodes(self, nodes_reach):
        """
        Removes a list/set of reach nodes.
        """
        for node_reach in nodes_reach:
            self.remove_reach_node(node_reach)

    def connect_to_parent_kripke_node(self, node_kripke_parent: 'KripkeNode'):
        """
        Establishes a connection between self and the parent Kripke node.
        """
        self.add_parent_kripke_node(node_kripke_parent)
        node_kripke_parent.add_child_kripke_node(self)

    def connect_to_parent_kripke_nodes(self, set_nodes_kripke_parent: Set['KripkeNode']):
        """
        Establishes connections between self and parent Kripke nodes.

        Only operates on the Kripke structure level (not on the reachable set level).
        """
        for node_kripke_parent in set_nodes_kripke_parent:
            self.connect_to_parent_kripke_node(node_kripke_parent)

    def connect_to_child_kripke_node(self, node_kripke_child: 'KripkeNode'):
        """
        Establishes a connection between self and the child Kripke node.
        """
        self.add_child_kripke_node(node_kripke_child)
        node_kripke_child.add_parent_kripke_node(self)

    def connect_to_child_kripke_nodes(self, set_nodes_kripke_child: Set['KripkeNode']):
        """
        Establishes connections between self and child Kripke nodes.

        Only operates on the Kripke structure level (not on the reachable set level).
        """
        for node_kripke_child in set_nodes_kripke_child:
            self.connect_to_child_kripke_node(node_kripke_child)

    def add_parent_kripke_node(self, node_parent: 'KripkeNode'):
        if node_parent not in self.list_nodes_kripke_parent:
            self.list_nodes_kripke_parent.append(node_parent)
            self.list_ids_nodes_kripke_parent.append(node_parent.id)

    def add_child_kripke_node(self, node_child: 'KripkeNode'):
        if node_child not in self.list_nodes_kripke_child:
            self.list_nodes_kripke_child.append(node_child)
            self.list_ids_nodes_kripke_child.append(node_child.id)

    def remove_parent_kripke_node(self, node_parent: 'KripkeNode'):
        if node_parent in self.list_nodes_kripke_parent:
            self.list_nodes_kripke_parent.remove(node_parent)
            self.list_ids_nodes_kripke_parent.remove(node_parent.id)

    def remove_child_kripke_node(self, node_child: 'KripkeNode'):
        if node_child in self.list_nodes_kripke_child:
            self.list_nodes_kripke_child.remove(node_child)
            self.list_ids_nodes_kripke_child.remove(node_child.id)


class KripkeStructure:
    """
    Kripke structure representing the state transitions of a reachability graph.
    """

    cls_bdd_dict: spot.impl.bdd_dict = None
    cls_set_propositions_relevant = set()

    def __init__(self, reach_interface: ReachableSetInterface = None):
        self.reach_interface = reach_interface
        self.step_end = self.reach_interface.step_end if reach_interface else None
        # an extra node as the final state of the kripke graph
        self.node_kripke_dead = None

        self.list_nodes_kripke: List[KripkeNode] = list()
        self.dict_step_to_propositions_to_kripke_nodes = defaultdict(lambda: defaultdict(list))
        self.dict_id_node_kripke_to_kripke_node: Dict[int, KripkeNode] = dict()
        self.dict_reach_node_to_kripke_node: Dict[ReachNode, KripkeNode] = dict()
        # only used for constructing twa
        self.dict_idx_state_twa_to_kripke_node = dict()
        # only used for constructing automaton graph
        self.dict_step_to_formula_proposition_to_kripke_node = defaultdict(dict)

        util_logger.print_and_log_info(logger, "* Constructing Kripke structure...")
        self._construct()
        util_logger.print_and_log_info(logger, f"\t#Nodes: {len(self.list_nodes_kripke)}")

    def _construct(self):
        """
        Constructs a Kripke structure.
        """
        self._construct_from_reachability_graph()
        self.update_mappings()
        self.establish_parent_child_relationship()

    def _construct_from_reachability_graph(self):
        """
        Constructs a Kripke structure from a reachability graph.

        A kripke node may contain one or multiple reachable nodes with the same set of propositions.
        The constructed Kripke structure is later used for model checking and extracting driving corridors.
        """
        KripkeNode.backend = "CPP" if self.reach_interface.config.reachable_set.mode_computation == 2 else "PYTHON"

        for step in range(self.step_end + 1):
            dict_proposition_holder_to_list_nodes_kripke = self.dict_step_to_propositions_to_kripke_nodes[step]

            for proposition_holder, list_nodes_reach in self.reach_interface.reachable_set_at_step(step).items():
                # determine relevant propositions of the reach nodes
                set_propositions_relevant = \
                    proposition_holder.propositions(include_temporary=False).intersection(
                        self.cls_set_propositions_relevant)
                set_propositions_relevant.add("true")
                set_propositions_relevant = frozenset(set_propositions_relevant)

                # retrieve existing/create new kripke node
                try:
                    node_kripke = dict_proposition_holder_to_list_nodes_kripke[set_propositions_relevant][0]
                    node_kripke.add_reach_nodes(set(list_nodes_reach))

                except (KeyError, IndexError):
                    node_kripke = KripkeNode(step, set_propositions_relevant, set(list_nodes_reach))
                    # add kripke to list
                    self.list_nodes_kripke.append(node_kripke)
                    # add kripke to dict
                    dict_proposition_holder_to_list_nodes_kripke[set_propositions_relevant].append(node_kripke)

        # create a dead kripke node at the end
        self.node_kripke_dead = KripkeNode(self.step_end + 1)
        self.node_kripke_dead.dead = True

    def update_mappings(self):
        """
        Updates internal mappings in the dictionaries.
        """
        # clear
        self.dict_step_to_propositions_to_kripke_nodes.clear()
        self.dict_id_node_kripke_to_kripke_node.clear()
        self.dict_reach_node_to_kripke_node.clear()

        # update
        for node_kripke in self.list_nodes_kripke:
            self.dict_step_to_propositions_to_kripke_nodes[node_kripke.step][node_kripke.set_propositions].append(
                node_kripke)
            self.dict_id_node_kripke_to_kripke_node[node_kripke.id] = node_kripke

            for node_reach in node_kripke.set_nodes_reach:
                self.dict_reach_node_to_kripke_node[node_reach] = node_kripke

    def _split_to_connected_components(self):
        """
        Split kripke nodes base on connected reach nodes therein.

        This prevents the kripke nodes from having disconnected reach nodes.
        """
        list_nodes_kripke_to_be_deleted = list()
        list_nodes_kripke_split = list()
        for node_kripke in self.list_nodes_kripke:
            list_lists_nodes_reach_connected = util_reach.determine_connected_components(
                list(node_kripke.set_nodes_reach))
            # all reach nodes are connected
            if not len(list_lists_nodes_reach_connected) > 1:
                continue

            # reach nodes form multiple connected components, split to new kripke nodes
            list_nodes_kripke_to_be_deleted.append(node_kripke)
            for list_nodes_reach_connected in list_lists_nodes_reach_connected:
                node_kripke_split = KripkeNode(node_kripke.step, node_kripke.set_propositions)
                # add reach nodes to kripke node
                node_kripke_split.add_reach_nodes(list_nodes_reach_connected)
                for node_reach in list_nodes_reach_connected:
                    self.dict_reach_node_to_kripke_node[node_reach] = node_kripke

                list_nodes_kripke_split.append(node_kripke_split)

        for node_kripke in list_nodes_kripke_to_be_deleted:
            self.remove_kripke_node(node_kripke)

        for node_kripke in list_nodes_kripke_split:
            self.add_kripke_node(node_kripke)

    def establish_parent_child_relationship(self):
        """
        Establish parent child relationship between kripke nodes.
        """
        # clear
        for node_kripke in self.list_nodes_kripke:
            node_kripke.set_nodes_reach_parent.clear()
            node_kripke.set_ids_nodes_reach_parent.clear()
            node_kripke.set_nodes_reach_child.clear()
            node_kripke.set_ids_nodes_reach_child.clear()

            node_kripke.list_nodes_kripke_parent.clear()
            node_kripke.list_nodes_kripke_child.clear()

        # establish
        for node_kripke in self.list_nodes_kripke:
            # add reach nodes to kripke node to update parent/child set information
            node_kripke.add_reach_nodes(frozenset(node_kripke.set_nodes_reach))

            # add connections to parent kripke nodes based on the parent-child relationship of reach nodes
            set_nodes_kripke_parent = set()
            for node_reach_parent in node_kripke.set_nodes_reach_parent:
                try:
                    set_nodes_kripke_parent.add(self.query_kripke_node_by_reach_node(node_reach_parent))

                except KeyError:
                    pass

            node_kripke.connect_to_parent_kripke_nodes(set_nodes_kripke_parent)

    def construct_twa(self):
        """
        Constructs a Transition-based Omega Automaton from the Kripke structure that is recognized by Spot.
        """
        # instantiate an automaton using the same BDD dict as the specification automaton
        automaton_model = spot.make_twa_graph(self.cls_bdd_dict)

        # register automaton propositions in BDD
        dict_proposition_to_bdd_proposition = {0: buddy.bddtrue,
                                               "dead": buddy.bdd_ithvar(automaton_model.register_ap('dead'))}
        for proposition in self.cls_set_propositions_relevant:
            dict_proposition_to_bdd_proposition[proposition] = \
                buddy.bdd_ithvar(automaton_model.register_ap(proposition))

        # create a state in the automaton for each node in the Kripke structure
        list_names_state = list()
        for node_kripke in self.list_nodes_kripke:
            # add a new state in the twa
            idx_state_twa = automaton_model.new_states(1)
            formula_proposition = util_spot.create_proposition_formula(node_kripke.set_propositions,
                                                                       dict_proposition_to_bdd_proposition,
                                                                       self.cls_set_propositions_relevant,
                                                                       dead=False)
            # update kripke node properties
            node_kripke.idx_state_twa = idx_state_twa
            node_kripke.formula_proposition = formula_proposition
            # update lists and dicts for indexing
            self.dict_idx_state_twa_to_kripke_node[idx_state_twa] = node_kripke
            self.dict_step_to_formula_proposition_to_kripke_node[node_kripke.step][formula_proposition] = node_kripke
            list_names_state.append(f"{node_kripke.step}_{node_kripke.id}")

        #  create a dead state in the end
        idx_state_twa_dead = automaton_model.new_states(1)
        formula_proposition_dead = util_spot.create_proposition_formula(None, dict_proposition_to_bdd_proposition,
                                                                        set(), dead=True)
        # update kripke node properties
        self.node_kripke_dead.idx_state_twa = idx_state_twa_dead
        self.node_kripke_dead.formula_proposition = formula_proposition_dead
        # update lists and dicts for indexing
        self.dict_idx_state_twa_to_kripke_node[idx_state_twa_dead] = self.node_kripke_dead
        self.dict_step_to_formula_proposition_to_kripke_node[self.node_kripke_dead.step][
            formula_proposition_dead] = self.node_kripke_dead
        list_names_state.append(f"{self.node_kripke_dead.step}_dead")

        # set initial state and state labels
        automaton_model.set_init_state(self.query_kripke_node_by_idx_state_twa(0).idx_state_twa)
        automaton_model.set_state_names(list_names_state)

        # add transitions between states. proposition formulas are labeled on the edge
        for node_kripke in self.list_nodes_kripke:
            for node_kripke_child in node_kripke.list_nodes_kripke_child:
                automaton_model.new_edge(node_kripke.idx_state_twa,
                                         node_kripke_child.idx_state_twa,
                                         node_kripke.formula_proposition)

            # connect to the dead state if the nodes are of the last step
            if node_kripke.step == self.step_end:
                automaton_model.new_edge(node_kripke.idx_state_twa,
                                         idx_state_twa_dead,
                                         node_kripke.formula_proposition)

        # add a self loop to the dead state
        automaton_model.new_edge(idx_state_twa_dead, idx_state_twa_dead, formula_proposition_dead)

        return automaton_model

    def kripke_nodes_at_step(self, step: int, merge: bool = False):
        if step not in self.dict_step_to_propositions_to_kripke_nodes:
            message = f"Given step {step} for kripke nodes retrieval is not valid."
            print(message)
            logger.warning(message)
            return []

        else:
            if not merge:
                return self.dict_step_to_propositions_to_kripke_nodes[step]

            else:
                # merge kripke nodes of different sets of propositions
                list_nodes_kripke = list()
                for list_nodes_kripke in self.dict_step_to_propositions_to_kripke_nodes[step].values():
                    list_nodes_kripke += list_nodes_kripke

                return list_nodes_kripke

    def kripke_nodes(self, merge: bool = False):
        if not merge:
            return self.dict_step_to_propositions_to_kripke_nodes

        else:
            dict_step_to_set_nodes_kripke = defaultdict(set)
            for step, dict_propositions_to_list_nodes_kripke in self.dict_step_to_propositions_to_kripke_nodes.items():
                for list_nodes_kripke in dict_propositions_to_list_nodes_kripke.values():
                    dict_step_to_set_nodes_kripke[step].update(set(list_nodes_kripke))

            return dict_step_to_set_nodes_kripke

    def reach_nodes_at_step(self, step: int, merge: bool = False):
        return self.reach_interface.reachable_set_at_step(step, merge)

    def query_kripke_node_by_id(self, id_node: int) -> Optional[KripkeNode]:
        return self.dict_id_node_kripke_to_kripke_node[id_node]

    def query_kripke_node_by_idx(self, idx_node: int) -> Optional[KripkeNode]:
        return self.list_nodes_kripke[idx_node]

    def query_kripke_node_by_reach_node(self, node_reach: ReachNode) -> Optional[KripkeNode]:
        return self.dict_reach_node_to_kripke_node[node_reach]

    def query_kripke_nodes_by_step_and_proposition(self, step: int,
                                                   proposition_holder: PropositionHolder) -> List[KripkeNode]:
        return self.dict_step_to_propositions_to_kripke_nodes[step][proposition_holder]

    def query_kripke_node_by_step_and_proposition_formula(self, step: int,
                                                          formula_proposition) -> Optional[KripkeNode]:
        return self.dict_step_to_formula_proposition_to_kripke_node[step][formula_proposition]

    def query_kripke_node_by_idx_state_twa(self, idx_state_twa) -> Optional[KripkeNode]:
        return self.dict_idx_state_twa_to_kripke_node[idx_state_twa]

    def add_kripke_node(self, node_kripke: KripkeNode):
        """
        Adds a kripke node into the structure.

        Operates on Kripke structure level.
        """
        self.list_nodes_kripke.append(node_kripke)
        self.dict_id_node_kripke_to_kripke_node[node_kripke.id] = node_kripke
        self.dict_step_to_propositions_to_kripke_nodes[node_kripke.step][node_kripke.set_propositions].append(
            node_kripke)
        for node_reach in node_kripke.set_nodes_reach:
            self.dict_reach_node_to_kripke_node[node_reach] = node_kripke

    def remove_kripke_node(self, node_kripke: KripkeNode):
        """
        Removes a kripke node from the structure.

        Operates on both reachability graph and Kripke structure level.
        """
        # parents
        for node_kripke_parent in node_kripke.list_nodes_kripke_parent:
            # remove references to reach nodes
            node_kripke_parent.set_nodes_reach_child -= node_kripke.set_nodes_reach
            node_kripke_parent.set_ids_nodes_reach_child -= node_kripke.set_ids_nodes_reach
            # remove reference to kripke node
            node_kripke_parent.remove_child_kripke_node(node_kripke)

        # children
        for node_kripke_child in node_kripke.list_nodes_kripke_child:
            # remove references to reach nodes
            node_kripke_child.set_nodes_reach_parent -= node_kripke.set_nodes_reach
            node_kripke_child.set_ids_nodes_reach_parent -= node_kripke.set_ids_nodes_reach
            # remove reference to kripke node
            node_kripke_child.remove_parent_kripke_node(node_kripke)

        # remove from other dictionaries
        try:
            self.dict_id_node_kripke_to_kripke_node.pop(node_kripke.id)
        except KeyError:
            pass

        for node_reach in node_kripke.set_nodes_reach:
            try:
                self.dict_reach_node_to_kripke_node.pop(node_reach)
            except KeyError:
                pass

        try:
            self.dict_step_to_propositions_to_kripke_nodes[node_kripke.step][node_kripke.set_propositions].remove(
                node_kripke)
        except ValueError:
            pass

        try:
            self.list_nodes_kripke.remove(node_kripke)
        except ValueError:
            pass
