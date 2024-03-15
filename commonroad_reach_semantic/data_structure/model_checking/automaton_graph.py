import copy
import logging
from collections import defaultdict
from typing import List, Dict, Set, Union

import spot
from commonroad_reach.utility import logger as util_logger

from commonroad_reach_semantic.data_structure.model_checking.kripke_node import KripkeNode
from commonroad_reach_semantic.data_structure.model_checking.kripke_structure import KripkeStructure
from commonroad_reach_semantic.utility import reach_operation as util_reach

logger = logging.getLogger(__name__)


class AutomatonNode:
    """
    Class representing a node in an automaton graph.
    """

    def __init__(self, idx_state_product: int, node_kripke: KripkeNode = None):
        self.idx_state_product = idx_state_product
        self.node_kripke: KripkeNode = node_kripke

        self.step = node_kripke.step if node_kripke else None
        self.formula_proposition = None

        self.set_nodes_auto_parent: Set[AutomatonNode] = set()
        self.set_nodes_auto_child: Set[AutomatonNode] = set()

        # for computing total number of runs
        self.counter = 0
        self.dict_partial_utilities = dict()

    def __repr__(self):
        return f"AutomatonNode(step={self.step},idx={self.idx_state_product},area={self.area},counter={self.counter})"

    def __key(self):
        return self.idx_state_product

    def __hash__(self):
        return hash(self.__key())

    def __eq__(self, other):
        return isinstance(other, AutomatonNode) and self.idx_state_product == other.idx_state_product

    def clone(self) -> 'AutomatonNode':
        node_kripke_cloned = self.node_kripke.clone()

        node_auto_cloned = AutomatonNode(self.idx_state_product, node_kripke_cloned)
        node_auto_cloned.formula_proposition = self.formula_proposition
        node_auto_cloned.set_nodes_auto_parent = self.set_nodes_auto_parent.copy()
        node_auto_cloned.set_nodes_auto_child = self.set_nodes_auto_child.copy()
        node_auto_cloned.counter = self.counter
        node_auto_cloned.dict_partial_utilities = copy.deepcopy(self.dict_partial_utilities)

        return node_auto_cloned

    def add_parent(self, node_parent: 'AutomatonNode'):
        self.set_nodes_auto_parent.add(node_parent)

    def remove_parent(self, node_parent: 'AutomatonNode'):
        self.set_nodes_auto_parent.discard(node_parent)

    def add_child(self, node_child: 'AutomatonNode'):
        self.set_nodes_auto_child.add(node_child)

    def remove_child(self, node_child: 'AutomatonNode'):
        self.set_nodes_auto_child.discard(node_child)

    @property
    def utility(self):
        # for obtaining the optimal driving corridor
        return sum(self.dict_partial_utilities.values())

    @property
    def area(self):
        return self.node_kripke.area if self.node_kripke else 0

    @property
    def p_lon_min(self):
        return self.node_kripke.p_lon_min

    @property
    def p_lon_max(self):
        return self.node_kripke.p_lon_max

    @property
    def p_lat_min(self):
        return self.node_kripke.p_lat_min

    @property
    def p_lat_max(self):
        return self.node_kripke.p_lat_max

    @property
    def v_lon_max(self):
        return self.node_kripke.v_lon_max

    @property
    def v_lon_min(self):
        return self.node_kripke.v_lon_min


# noinspection PyTypeChecker
class AutomatonGraph:
    """
    Class representing an automaton as a state-based graph.
    """

    def __init__(self, automaton_product: spot.twa_graph, kripke_structure: KripkeStructure):
        # product automaton considering all LTL formulas
        self.automaton_product = automaton_product
        self.kripke_structure = kripke_structure
        self.step_end = self.kripke_structure.step_end

        self.list_nodes_auto: List[AutomatonNode] = list()
        self.dict_idx_state_product_to_node_auto: Dict[int, AutomatonNode] = dict()
        self.dict_step_to_set_nodes_auto: Dict[int, Set[AutomatonNode]] = defaultdict(set)
        self.dict_node_kripke_to_set_nodes_auto: Dict[KripkeNode] = defaultdict(set)
        self.has_accepting_run = False
        self.set_maneuvers = set()

        self._construct_graph()
        self._update_mappings()
        self._establish_parent_child_relationship()
        self._update_mappings()
        self._update_kripke_nodes()

        self._prune_graph()
        self._split_to_connected_components()

    @property
    def num_nodes_reach(self):
        sum_nodes_reach = 0

        for node_auto in self.list_nodes_auto:
            sum_nodes_reach += len(node_auto.node_kripke.set_nodes_reach)

        return sum_nodes_reach

    def _construct_graph(self):
        """
        Constructs a graph from the given product automaton.
        """
        # iterate through states in the product automaton and create nodes accordingly
        for idx_state_product in range(self.automaton_product.num_states()):
            node_auto = AutomatonNode(idx_state_product)
            # retrieve the proposition formula of the node
            # all outgoing edges have the same proposition formula, so here we just take the first edge
            list_edges_outgoing = list(self.automaton_product.out(node_auto.idx_state_product))
            node_auto.formula_proposition = list_edges_outgoing[0].cond

            # add to lists and dicts for indexing
            self.list_nodes_auto.append(node_auto)

            # examine accepting status
            if self.automaton_product.state_is_accepting(idx_state_product):
                self.has_accepting_run = True

        self.list_nodes_auto[0].step = 0

        # abort further construction if there is no accepting run
        if not self.has_accepting_run:
            util_logger.print_and_log_info(logger, "No accepting run in the product automaton.")

    def _update_mappings(self):
        """
        Updates internal mappings in the dictionaries.
        """
        # clear
        self.dict_idx_state_product_to_node_auto.clear()
        self.dict_step_to_set_nodes_auto.clear()

        # update
        for node_auto in self.list_nodes_auto:
            self.dict_idx_state_product_to_node_auto[node_auto.idx_state_product] = node_auto
            self.dict_step_to_set_nodes_auto[node_auto.step].add(node_auto)

    def _establish_parent_child_relationship(self):
        """
        Establish parent child relationship between automaton nodes.
        """
        # clear
        for node_auto in self.list_nodes_auto:
            node_auto.set_nodes_auto_parent.clear()
            node_auto.set_nodes_auto_child.clear()

        # establish
        formula_proposition_dead = self.kripke_structure.node_kripke_dead.formula_proposition
        for node_auto in self.list_nodes_auto:
            list_edges_outgoing = list(self.automaton_product.out(node_auto.idx_state_product))
            # connect to child nodes
            for edge_outgoing in list_edges_outgoing:
                idx_state_product_child = edge_outgoing.dst
                node_auto_child = self.query_automaton_node_by_idx_state_product(idx_state_product_child)

                node_auto.add_child(node_auto_child)
                node_auto_child.add_parent(node_auto)

                # set step
                if node_auto.formula_proposition != formula_proposition_dead:
                    node_auto_child.step = node_auto.step + 1

                else:
                    node_auto_child.step = node_auto.step

    def _update_kripke_nodes(self):
        """
        Updates the kripke nodes associated with automaton nodes.
        """
        for node_auto in self.list_nodes_auto:
            node_kripke = \
                self.kripke_structure.query_kripke_node_by_step_and_proposition_formula(node_auto.step,
                                                                                        node_auto.formula_proposition)
            node_auto.node_kripke = node_kripke
            # one kripke node may map to multiple automaton nodes
            self.dict_node_kripke_to_set_nodes_auto[node_kripke].add(node_auto)

    def _prune_graph(self):
        """
        Prunes reach nodes that don't have a parent in the kripke nodes along the accepting runs.

        The accepting kripke nodes form a subgraph of the reachability graph, whose reach nodes might not have a
        parent in the kripke nodes along the accepting runs. These unreachable reach nodes are removed.
        """
        if not self.has_accepting_run:
            return

        set_nodes_reach_acc_prev = set()
        set_nodes_reach_acc = set()

        for step, set_nodes_auto in self.dict_step_to_set_nodes_auto.items():
            set_nodes_reach_acc.clear()
            # add reach nodes of the initial step into set of accepting reach nodes
            if step == 0:
                for node_auto in set_nodes_auto:
                    set_nodes_reach_acc.update(node_auto.node_kripke.set_nodes_reach)
                    continue

            else:
                for node_auto in set_nodes_auto:
                    set_nodes_reach_to_delete = set()

                    for node_reach in node_auto.node_kripke.set_nodes_reach:
                        # discard if the parent nodes don't intersect with the accepted nodes of the previous step
                        if set(node_reach.list_nodes_parent).intersection(set_nodes_reach_acc_prev):
                            set_nodes_reach_acc.add(node_reach)

                        else:
                            set_nodes_reach_to_delete.add(node_reach)

                    node_auto.node_kripke.remove_reach_nodes(set_nodes_reach_to_delete)

            set_nodes_reach_acc_prev = set_nodes_reach_acc.copy()

    def _split_to_connected_components(self):
        """
        Splits automaton nodes (and their associated kripke nodes) to connected components of reach nodes.

        This prevents automaton nodes (and kripke nodes) from having disconnected reach nodes.
        """
        for step in range(self.step_end):
            set_nodes_auto = self.query_automaton_nodes_by_step(step)

            list_nodes_auto_to_be_deleted = list()
            list_nodes_auto_split = list()
            list_nodes_kripke_split = list()

            for node_auto in set_nodes_auto:
                node_kripke = node_auto.node_kripke

                # skip if the automaton node has already been processed
                if node_auto in list_nodes_auto_to_be_deleted:
                    continue

                # determine the number of connected components based on reach nodes in the kripke node
                list_lists_nodes_reach_connected = \
                    util_reach.determine_connected_components(list(node_kripke.set_nodes_reach))

                # skip if all reach nodes are connected
                if not len(list_lists_nodes_reach_connected) > 1:
                    continue

                # a kripke node may map to multiple automaton nodes, process them together
                set_nodes_auto_relevant = self.dict_node_kripke_to_set_nodes_auto[node_kripke]
                list_nodes_auto_to_be_deleted += list(set_nodes_auto_relevant)

                # reach nodes form multiple connected components, split to new kripke node
                for idx_cc, list_nodes_reach_connected in enumerate(list_lists_nodes_reach_connected):
                    node_kripke_split = KripkeNode(node_kripke.step, node_kripke.set_propositions,
                                                   set(list_nodes_reach_connected))
                    list_nodes_kripke_split.append(node_kripke_split)

                # split automaton nodes, each connecting to one kripke node
                for node_auto_relevant in set_nodes_auto_relevant:
                    for idx, node_kripke_split in enumerate(list_nodes_kripke_split):
                        # prevent colliding indices
                        idx_new = node_auto_relevant.idx_state_product * 1000 + idx
                        node_auto_split = AutomatonNode(idx_new, node_kripke_split)

                        list_nodes_auto_split.append(node_auto_split)
                        self.dict_node_kripke_to_set_nodes_auto[node_auto_split].add(node_auto_split)

                        # update parent-child relationship of new automaton nodes
                        for node_auto_parent in node_auto_relevant.set_nodes_auto_parent:
                            node_auto_parent.add_child(node_auto_split)
                            node_auto_split.add_parent(node_auto_parent)

                        for node_auto_child in node_auto_relevant.set_nodes_auto_child:
                            node_auto_child.add_parent(node_auto_split)
                            node_auto_split.add_child(node_auto_child)

            # remove auto nodes
            for node_auto_delete in list_nodes_auto_to_be_deleted:
                self.remove_auto_node(node_auto_delete)

            # add split auto nodes
            for node_auto_split in list_nodes_auto_split:
                self.add_auto_node(node_auto_split)

        # re-establish parent-child relationship
        self.kripke_structure.establish_parent_child_relationship()

    def add_auto_node(self, node_auto: AutomatonNode):
        """
        Adds an automaton node.
        """
        if node_auto not in self.list_nodes_auto:
            # add auto node
            self.list_nodes_auto.append(node_auto)
            self.dict_idx_state_product_to_node_auto[node_auto.idx_state_product] = node_auto
            self.dict_step_to_set_nodes_auto[node_auto.step].add(node_auto)
            self.dict_node_kripke_to_set_nodes_auto[node_auto.node_kripke].add(node_auto)

            # add associated kripke node
            self.kripke_structure.add_kripke_node(node_auto.node_kripke)

    def remove_auto_node(self, node_auto: AutomatonNode):
        """
        Removes an automaton node.
        """
        if node_auto in self.list_nodes_auto:
            # update parent-child relationship
            for node_auto_parent in node_auto.set_nodes_auto_parent:
                node_auto_parent.remove_child(node_auto)

            for node_auto_child in node_auto.set_nodes_auto_child:
                node_auto_child.remove_parent(node_auto)

            # remove auto node
            self.list_nodes_auto.remove(node_auto)
            self.dict_idx_state_product_to_node_auto.pop(node_auto.idx_state_product)
            self.dict_step_to_set_nodes_auto[node_auto.step].remove(node_auto)
            self.dict_node_kripke_to_set_nodes_auto[node_auto.node_kripke].remove(node_auto)

            # remove kripke node associated to the automaton node
            self.kripke_structure.remove_kripke_node(node_auto.node_kripke)

    def query_automaton_node_by_idx_state_product(self, idx_state_product: int) -> AutomatonNode:
        return self.dict_idx_state_product_to_node_auto[idx_state_product]

    def query_automaton_nodes_by_step(self, step: int) -> Set[AutomatonNode]:
        set_nodes_auto = set()
        for node_automaton in self.dict_step_to_set_nodes_auto[step]:
            set_nodes_auto.add(node_automaton)

        return set_nodes_auto

    def retrieve_accepting_kripke_nodes(self, merge: bool = False) -> Union[Dict[int, Set[KripkeNode]],
    List[KripkeNode]]:
        """
        Returns accepting kripke nodes from the product automaton.

        Note that the result is the union of kripke nodes along accepting runs, which cannot be directly used to
        obtain specification-compliant driving corridors.
        """
        if merge:
            return [node_auto.node_kripke for node_auto in self.list_nodes_auto]

        else:
            dict_step_to_set_nodes_kripke_accepting = defaultdict(set)
            for step in range(self.step_end + 1):
                set_nodes_auto = self.query_automaton_nodes_by_step(step)
                for node_auto in set_nodes_auto:
                    dict_step_to_set_nodes_kripke_accepting[node_auto.step].add(node_auto.node_kripke)

            return dict_step_to_set_nodes_kripke_accepting
