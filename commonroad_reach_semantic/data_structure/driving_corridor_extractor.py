import logging
import random
import time
from collections import defaultdict
from typing import Set, List

import commonroad_reach.utility.logger as util_logger
import networkx as nx
import numpy as np
from commonroad_reach.data_structure.reach.reach_interface import ReachableSetInterface

import commonroad_reach_semantic.utility.spot as util_spot
from commonroad_reach_semantic.data_structure.config.semantic_configuration import SemanticConfiguration
from commonroad_reach_semantic.data_structure.model_checking.automaton_graph import AutomatonNode
from commonroad_reach_semantic.data_structure.model_checking.kripke_node import KripkeNode
from commonroad_reach_semantic.data_structure.model_checking.spot_interface import SpotInterface

logger = logging.getLogger(__name__)


class DrivingCorridor:
    """
    Class representing a driving corridor in the automaton graph.
    """
    cnt_id = 0

    def __init__(self):
        self.id = DrivingCorridor.cnt_id
        DrivingCorridor.cnt_id += 1

        # automaton-node-related
        self.list_nodes_auto: List[AutomatonNode] = list()
        self.list_idx_state_auto: List[int] = list()
        self.set_nodes_auto_child = set()

        # kripke-node-related
        self.list_nodes_kripke: List[KripkeNode] = list()

    def __repr__(self):
        return f"DrivingCorridor(id={self.id})"

    def __eq__(self, other):
        return isinstance(other, self.__class__) and self.list_idx_state_auto == other.list_idx_state_auto

    def __key(self):
        return tuple(self.list_idx_state_auto)

    def __hash__(self):
        return hash(self.__key())

    @property
    def utility(self):
        sum_utility = 0
        for node_auto in self.list_nodes_auto:
            sum_utility += node_auto.utility

        return sum_utility

    @property
    def area(self):
        """
        Summed area of automaton nodes.
        """
        sum_area_nodes_auto = 0
        for node_auto in self.list_nodes_auto:
            sum_area_nodes_auto += node_auto.area

        return sum_area_nodes_auto

    @property
    def p_lon_max(self):
        """
        Maximum position of the final reach nodes in the longitudinal direction.
        """
        p_lon_max = -np.infty
        node_kripke_final = self.list_nodes_kripke[-1]
        for node_reach in node_kripke_final.set_nodes_reach:
            p_lon_max = max(p_lon_max, node_reach.p_lon_max)

        return p_lon_max

    @classmethod
    def reset_id_counter(cls):
        cls.cnt_id = 0

    def print(self):
        for node_kripke in self.list_nodes_kripke:
            print(f"{node_kripke.step}: ")
            for proposition in node_kripke.set_propositions:
                print(f"\t\t{proposition}")

    def clone(self):
        """
        Return a cloned driving corridor.
        """
        corridor_cloned = DrivingCorridor()
        corridor_cloned.list_nodes_auto = self.list_nodes_auto.copy()
        corridor_cloned.list_idx_state_auto = self.list_idx_state_auto.copy()
        corridor_cloned.set_nodes_auto_child = self.set_nodes_auto_child.copy()

        corridor_cloned.list_nodes_kripke = self.list_nodes_kripke.copy()

        return corridor_cloned

    def extend(self, node_auto: AutomatonNode):
        """
        Extends driving corridor with an automaton node.
        """
        self.list_nodes_auto.append(node_auto)
        self.list_idx_state_auto.append(node_auto.idx_state_product)
        self.set_nodes_auto_child = node_auto.set_nodes_auto_child

        self.list_nodes_kripke.append(node_auto.node_kripke)

    def clone_and_extend(self, node_auto: AutomatonNode):
        """
        Returns a cloned and extended driving corridor.
        """
        corridor_extended = self.clone()
        corridor_extended.extend(node_auto)

        return corridor_extended

    def deep_copy(self) -> 'DrivingCorridor':
        """
        Makes a deep copy of the corridor.

        Native deep copy is not supported due to the use of Pybind11.
        """
        corridor_cloned = DrivingCorridor()

        for node_auto in self.list_nodes_auto:
            node_auto_cloned = node_auto.clone()
            corridor_cloned.list_nodes_auto.append(node_auto_cloned)
            corridor_cloned.list_nodes_kripke.append(node_auto_cloned.node_kripke)

        for node_auto_child in self.set_nodes_auto_child:
            corridor_cloned.set_nodes_auto_child.add(node_auto_child.clone())

        return corridor_cloned

    def retrieve_kripke_nodes(self, merge: bool = False):
        """
        Returns the kripke nodes associated with the driving corridor.
        """
        if merge:
            return self.list_nodes_kripke.copy()

        else:
            dict_step_to_nodes_kripke = defaultdict(set)
            for node_kripke in self.list_nodes_kripke:
                dict_step_to_nodes_kripke[node_kripke.step].add(node_kripke)

            return dict_step_to_nodes_kripke

    @classmethod
    def from_node(cls, node_auto: AutomatonNode):
        """
        Returns a driving corridor with the input automaton node as the initial node.
        """
        corridor = DrivingCorridor()
        corridor.extend(node_auto)

        return corridor

    @classmethod
    def from_nodes(cls, list_nodes_auto: List[AutomatonNode]):
        """
        Returns a driving corridor with the input automaton nodes as the initial nodes.
        """
        corridor = DrivingCorridor()
        for node_auto in list_nodes_auto:
            corridor.extend(node_auto)

        return corridor


class DrivingCorridorExtractor:
    """
    Class to extract specification-compliant driving corridors within reachable sets.
    """
    cls_w_area: float = 1.0
    cls_w_progression: float = 1.0
    cls_w_velocity: float = 1.0
    cls_w_reference: float = 1.0
    cls_w_deviation: float = 1.5

    def __init__(self, spot_interface: SpotInterface):
        self.config: SemanticConfiguration = spot_interface.config
        self.spot_interface: SpotInterface = spot_interface
        self.reach_interface: ReachableSetInterface = spot_interface.reach_interface
        self.kripke_structure = spot_interface.kripke_structure
        self.graph_automaton = spot_interface.graph_automaton
        self.step_end = self.reach_interface.step_end

        self.flag_search = True
        self.set_corridors_extracted: Set[DrivingCorridor] = set()
        self.area_max: float = -np.infty

        self.area_max: float = -np.infty

        logger.info("Driving corridor extractor created.")

    def extract_corridors(self, search: bool = True):
        """
        Extracts driving corridors from the product automaton.
        """
        if not self.spot_interface.graph_automaton.has_accepting_run:
            util_logger.print_and_log_info(logger, "No specification-compliant driving corridor.")
            return None

        time_start = time.time()
        util_logger.print_and_log_info(logger, "* Extracting driving corridors...")

        self.flag_search = search
        if not search:
            num_corridors_max = self.config.reachable_set.num_corridors_max
            num_paths = util_spot.compute_num_paths_in_graph(self.graph_automaton)
            util_logger.print_and_log_info(logger, f"\t#Driving corridor:")
            util_logger.print_and_log_info(logger, f"\t\t#Paths in graph: {num_paths}")

            if num_paths < num_corridors_max:
                # extract all paths in the graph
                self.set_corridors_extracted = self._extract_all_driving_corridors()

            else:
                # draw samples in graph
                self.set_corridors_extracted = self._sample_driving_corridors(num_corridors_max)

        else:
            # determines the utility of each node in the automaton graph
            self.area_max = max([node_auto.area for node_auto in self.graph_automaton.list_nodes_auto])
            for node in self.graph_automaton.list_nodes_auto:
                self._determine_utility_of_automaton_node(node)

            self.area_max = max([node_auto.area for node_auto in self.graph_automaton.list_nodes_auto])
            for node in self.graph_automaton.list_nodes_auto:
                self._determine_utility_of_automaton_node(node)
            # perform graph search to retrieve the optimal path
            self.set_corridors_extracted = self._search_optimal_driving_corridors()

        self.set_corridors_extracted = self._prune_driving_corridors(self.set_corridors_extracted)

        time_computation = time.time() - time_start
        util_logger.print_and_log_info(logger, f"\tTook: \t{time_computation:.3f}s")

    def _determine_utility_of_automaton_node(self, node_auto: AutomatonNode):
        """
        Determines the utilities of the automaton node.

        The utility of a node is the weighted sum of u_area, u_progression, u_velocity, and u_reference.
        All utilities are normalized to [0,1]

        u_area: the normalized utility areas of the associated reach nodes,
        u_velocity: the normalized utility of the velocity in the longitudinal direction
        u_progression: the normalized utility of the progression in the longitudinal direction
        u_reference: the normalized deviation from the reference path
        """
        self._determine_area_utility(node_auto)
        self._determine_velocity_utility(node_auto)
        self._determine_progression_utility(node_auto)
        self._determine_reference_path_utility(node_auto)

    def _determine_area_utility(self, node_auto: AutomatonNode):
        u_area = node_auto.area / self.area_max
        node_auto.dict_partial_utilities["u_area"] = self.cls_w_area * u_area

    def _determine_velocity_utility(self, node_auto: AutomatonNode):
        a_lon_max = self.config.vehicle.ego.a_lon_max
        v_lon_init = self.config.planning.v_lon_initial

        t = self.config.planning.dt * self.config.planning.steps_computation
        v_lon_max = a_lon_max * t + v_lon_init
        d_v_lon_max = v_lon_max - v_lon_init
        v_lon_mean_node = (node_auto.v_lon_max + node_auto.v_lon_min) / 2

        u_velocity = (v_lon_mean_node - v_lon_init) / d_v_lon_max
        node_auto.dict_partial_utilities["u_velocity"] = self.cls_w_velocity * u_velocity

    def _determine_progression_utility(self, node_auto):
        a_lon_max = self.config.vehicle.ego.a_lon_max
        v_lon_init = self.config.planning.v_lon_initial
        p_lon_init = self.config.planning.p_lon_initial

        t = self.config.planning.dt * self.config.planning.steps_computation
        p_lon_max = 0.5 * a_lon_max * t ** 2 + v_lon_init * t + p_lon_init
        d_p_lon_max = p_lon_max - p_lon_init
        p_lon_mean_node = (node_auto.p_lon_max + node_auto.p_lon_min) / 2

        u_progression = (p_lon_mean_node - p_lon_init) / d_p_lon_max
        node_auto.dict_partial_utilities["u_progression"] = self.cls_w_progression * u_progression

    def _determine_reference_path_utility(self, node_auto):
        if node_auto.p_lat_min > 0:
            p_ref = node_auto.p_lat_min

        elif node_auto.p_lat_max < 0:
            p_ref = abs(node_auto.p_lat_max)

        else:
            p_ref = 0

        u_reference = np.exp(-self.cls_w_deviation * p_ref)
        node_auto.dict_partial_utilities["u_reference"] = self.cls_w_reference * u_reference

    @staticmethod
    def _verify_corridor(corridor: DrivingCorridor):
        """
        Verifies the validity of the input corridor.

        Check naughty ones in the Kripke nodes (reach nodes with no parent nodes)
        """
        dict_step_to_nodes_reach = dict()
        for node_kripke in corridor.list_nodes_kripke:
            dict_step_to_nodes_reach[node_kripke.step] = node_kripke.set_nodes_reach

        list_steps = list(reversed(list(dict_step_to_nodes_reach.keys())))
        for step in list_steps[:-1]:
            set_nodes_reach_current = dict_step_to_nodes_reach[step]
            set_nodes_reach_previous = set(dict_step_to_nodes_reach[step - 1])

            for node_reach in set_nodes_reach_current:
                if set(node_reach.list_nodes_parent).isdisjoint(set_nodes_reach_previous):
                    print(f"Caught a naughty corridor with id {corridor.id} at step {step}")

    def _extract_all_driving_corridors(self):
        """
        Extracts all driving corridors in the graph.
        """
        util_logger.print_and_log_info(logger, f"\tExtracting all corridors...")

        set_corridors_extracted: Set[DrivingCorridor] = set()
        for step in range(self.step_end + 1):
            # initialize list of driving corridors from the initial automaton nodes
            if step == 0:
                set_nodes_auto_initial = self.graph_automaton.query_automaton_nodes_by_step(step)
                set_corridors_extracted = {DrivingCorridor.from_node(node_auto) for node_auto in set_nodes_auto_initial}

            else:
                set_corridors_extended = set()

                for corridor in set_corridors_extracted:
                    for node_auto_child in corridor.set_nodes_auto_child:
                        set_corridors_extended.add(corridor.clone_and_extend(node_auto_child))

                set_corridors_extracted = set_corridors_extended

        return set_corridors_extracted

    def _sample_driving_corridors(self, num_samples: int):
        """
        Draws samples of driving corridors in the automaton graph.
        """
        util_logger.print_and_log_info(logger, f"\tSampling {num_samples} corridors...")

        set_corridors = set()
        for _ in range(num_samples):
            # start from the initial automaton node
            node_auto = self.graph_automaton.list_nodes_auto[0]
            corridor = DrivingCorridor.from_node(node_auto)

            while node_auto.step < self.step_end:
                num_children = len(list(node_auto.set_nodes_auto_child))
                node_auto_next = list(node_auto.set_nodes_auto_child)[random.randint(0, num_children - 1)]

                corridor.extend(node_auto_next)
                node_auto = node_auto_next

            set_corridors.add(corridor)

        return set_corridors

    def _search_optimal_driving_corridors(self):
        """
        Returns the optimal driving corridor with the highest utility in the automaton graph.

        We used a modified version of Dijkstra search to find the "longest" path in a graph.
        """
        util_logger.print_and_log_info(logger, f"\tSearching for optimal corridor...")

        set_corridors = set()

        # convert into a networkx graph
        graph_nx = util_spot.convert_automaton_graph_to_networkx_graph(self.graph_automaton)

        # add weights to edges: rewrite weights to find the 'longest' path with Dijkstra
        utility_max = max([node_auto.utility for node_auto in self.graph_automaton.list_nodes_auto])

        for node_auto in self.graph_automaton.list_nodes_auto:
            for node_auto_child in node_auto.set_nodes_auto_child:
                weight_modified = utility_max - node_auto.utility
                graph_nx[node_auto.idx_state_product][node_auto_child.idx_state_product]['weight'] = weight_modified

        # compute the shortest path from the root node to the target nodes, based on the modified weight
        node_auto_source = self.graph_automaton.list_nodes_auto[0]
        # set each of the automaton nodes in the final step as the target
        for node_auto_target in self.graph_automaton.query_automaton_nodes_by_step(self.step_end):
            list_idx_state_product_on_path = nx.shortest_path(graph_nx,
                                                              source=node_auto_source.idx_state_product,
                                                              target=node_auto_target.idx_state_product,
                                                              weight='weight')
            # query automaton nodes on the path
            list_nodes_auto_on_path = [self.graph_automaton.query_automaton_node_by_idx_state_product(idx_state_product)
                                       for idx_state_product in list_idx_state_product_on_path]

            # create corridor
            set_corridors.add(DrivingCorridor.from_nodes(list_nodes_auto_on_path))

        return set_corridors

    def _prune_driving_corridors(self, set_corridors: Set[DrivingCorridor]):
        """
        Prunes driving corridors by removing unreachable reach nodes.

        A corridor forms a subgraph of the reachability graph, whose reach nodes might not have a parent along the
        corridor. These unreachable reach nodes are removed.
        """
        # deep copy is required to not affect other corridors since they might reference the same auto/kripke nodes.
        # TODO: this is not a proper deep copy!
        set_corridors_pruned = {corridor.clone() for corridor in set_corridors}
        set_corridors_keep = set()
        set_nodes_auto_corridors_keep = set()

        for corridor in set_corridors_pruned:
            set_nodes_reach_acc_prev = set()
            set_nodes_reach_acc = set()

            for node_auto in corridor.list_nodes_auto:
                set_nodes_reach_acc.clear()
                # add reach nodes of the initial step into set of accepting reach nodes
                if node_auto.step == 0:
                    set_nodes_reach_acc.update(node_auto.node_kripke.set_nodes_reach)

                else:
                    set_nodes_reach_to_delete = set()

                    for node_reach in node_auto.node_kripke.set_nodes_reach:
                        # discard if the parent nodes don't intersect with the accepted nodes of the previous step
                        if set(node_reach.list_nodes_parent).intersection(set_nodes_reach_acc_prev):
                            set_nodes_reach_acc.add(node_reach)

                        else:
                            set_nodes_reach_to_delete.add(node_reach)

                    node_auto.node_kripke.remove_reach_nodes(set_nodes_reach_to_delete)

                set_nodes_reach_acc_prev = set_nodes_reach_acc.copy()

            # only keep the ones still reaching the final step
            reached_final_step = len(corridor.list_nodes_auto[-1].node_kripke.set_nodes_reach) > 0
            if reached_final_step:
                set_corridors_keep.add(corridor)
                set_nodes_auto_corridors_keep.update(set(corridor.list_nodes_auto))

        # recalculate the utility of automaton nodes of corridors
        if set_nodes_auto_corridors_keep:
            self.area_max = max([node_auto.area for node_auto in set_nodes_auto_corridors_keep])
            for corridor in set_corridors_keep:
                for node_auto in corridor.list_nodes_auto:
                    self._determine_utility_of_automaton_node(node_auto)

        return set_corridors_keep

    def determine_optimal_corridor(self):
        """
        Returns the driving corridor with the optimal utility.
        """
        list_corridors = list(self.set_corridors_extracted)
        list_corridors.sort(key=lambda corridor: corridor.utility, reverse=True)
        corridor_optimal = list_corridors[0]

        print(f"\t\tMax utility of DCs: {corridor_optimal.utility:.6}")

        return corridor_optimal
