import logging
from collections import defaultdict
from typing import List, Union

import networkx as nx
from commonroad.scenario.lanelet import Lanelet
from commonroad_reach import pycrreach
from commonroad_reach.data_structure.reach.reach_node import ReachNode
from commonroad_reach.utility import logger as util_logger

from commonroad_reach_semantic.data_structure.environment_model.position_interval import PositionInterval

logger = logging.getLogger(__name__)


def are_intersecting_lanelets(lanelet_1: Lanelet, lanelet_2: Lanelet) -> bool:
    """
    Returns True if the two lanelets are intersecting.
    """
    id_lanelet_1 = lanelet_1.lanelet_id
    id_lanelet_2 = lanelet_2.lanelet_id

    if id_lanelet_1 != id_lanelet_2 and \
            id_lanelet_1 not in lanelet_2.predecessor and \
            id_lanelet_1 not in lanelet_2.successor and \
            id_lanelet_1 != lanelet_2.adj_left and \
            id_lanelet_1 != lanelet_2.adj_right and \
            lanelet_1.polygon.shapely_object.buffer(-0.05).intersects(lanelet_2.polygon.shapely_object.buffer(-0.05)):
        return True

    return False


def split_reach_node_to_interval(node: ReachNode, interval: PositionInterval, direction: str):
    """
    Returns a reach node adapted to the input position interval.
    """
    node_split = node.clone()

    if direction == "lon":
        node_split.polygon_lon = node_split.polygon_lon.intersect_halfspace(1, 0, interval.p_max)
        node_split.polygon_lon = node_split.polygon_lon.intersect_halfspace(-1, 0, -interval.p_min)

    elif direction == "lat":
        node_split.polygon_lat = node_split.polygon_lat.intersect_halfspace(1, 0, interval.p_max)
        node_split.polygon_lat = node_split.polygon_lat.intersect_halfspace(-1, 0, -interval.p_min)
    else:
        util_logger.print_and_log_error(f"Invalid direction: {direction} (valid: \"lat\"/\"lon\"")
        return None

    # check validity of the split polygons
    if node_split and node_split.polygon_lon and node_split.polygon_lat:
        return node_split
    else:
        return None


def discard_nodes_with_short_edge(list_nodes: List[ReachNode], length: float):
    """
    Discards nodes with an edge shorter than the specified length.
    """
    return [
        node for node in list_nodes
        if node.p_lon_max - node.p_lon_min >= length and node.p_lat_max - node.p_lat_min >= length
    ]


def determine_connected_components(list_nodes_reach):
    """
    Determines and returns the connected reachable sets in the position domain.
    """
    dict_adjacency = determine_connected_reach_nodes(list_nodes_reach)

    # adjacency list: list with tuples, e.g., (0, 1) represents that node 0 and node 1 are connected
    set_tuples_adjacent = set()
    for list_tuples_adjacent in dict_adjacency.values():
        set_tuples_adjacent.update(set(list_tuples_adjacent))

    list_lists_nodes_connected = list()
    # create graph with nodes = reach nodes and edges = adjacency status
    graph = nx.Graph()
    graph.add_nodes_from(list(range(len(list_nodes_reach))))
    graph.add_edges_from(set_tuples_adjacent)

    for set_indices_nodes_reach_connected in nx.connected_components(graph):
        list_nodes_reach_connected = [list_nodes_reach[idx] for idx in set_indices_nodes_reach_connected]
        list_lists_nodes_connected.append(list_nodes_reach_connected)

    return list_lists_nodes_connected


def determine_connected_reach_nodes(list_nodes_reach: Union[List[ReachNode], List[pycrreach.ReachNode]]):
    """
    Determines connected sets in the position domain.

    Returns a dictionary in the form of {node index self:list of tuples (node index self, node index other)}.
    This function is the equivalent python function to pycrreach.connected_reachset_boost().
    """
    dict_adjacency = defaultdict(list)

    if not list_nodes_reach:
        return dict_adjacency

    list_position_rectangles = [node_reach.position_rectangle for node_reach in list_nodes_reach]

    # iterate over all rectangles
    for idx1, position_rect_1 in enumerate(list_position_rectangles):
        for idx2, position_rect_2 in enumerate(list_position_rectangles):
            if idx1 == idx2:
                continue

            if position_rect_1.intersects(position_rect_2):
                dict_adjacency[idx1].append((idx1, idx2))

    return dict_adjacency
