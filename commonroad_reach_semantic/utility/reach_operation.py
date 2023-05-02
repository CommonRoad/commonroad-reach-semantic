import logging
from collections import defaultdict
from typing import List, Optional, Union

import networkx as nx
from commonroad.scenario.lanelet import Lanelet
from commonroad_reach.data_structure.reach.reach_node import ReachNode
from commonroad_reach.data_structure.reach.reach_polygon import ReachPolygon
from commonroad_reach.utility import geometry as util_geometry
from commonroad_reach.utility import logger as util_logger

from commonroad_reach_semantic import pycrreachs
from commonroad_reach_semantic.data_structure.environment_model.position_interval import PositionInterval
from commonroad_reach_semantic.data_structure.rule.proposition import PropositionGroup as PropGroup
from commonroad_reach_semantic.data_structure.reach.semantic_reach_node import SemanticReachNode

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


def split_reach_node_to_interval(node: SemanticReachNode, interval: PositionInterval, direction: str):
    """
    Returns a reach node adapted to the input position interval.
    """
    node_split = node.clone()
    node_split.proposition_holder.add_propositions(interval.set_propositions, PropGroup.POSITION)

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


def construct_reach_nodes(drivable_area: List[ReachPolygon],
                          list_propagated_set: List[SemanticReachNode]) -> List[SemanticReachNode]:
    """
    Constructs nodes of the reachability graph.

    The nodes are constructed by intersecting propagated sets with the drivable areas to determine the reachable
    positions and velocities.

    Steps:
        1. examine the adjacency of drivable areas and the propagated sets. They are considered adjacent if they
           overlap in the position domain.
        2. create a node from each drivable area and its adjacent propagated sets.
    """
    reachable_set = []

    list_rectangles_propagated_set = [propagated_set.position_rectangle for propagated_set in list_propagated_set]
    list_rectangles_drivable_area = drivable_area
    dict_rectangle_adjacency = util_geometry.create_adjacency_dictionary(list_rectangles_drivable_area,
                                                                         list_rectangles_propagated_set)

    for idx_drivable_area, list_idx_propagated_sets_adjacent in dict_rectangle_adjacency.items():
        rectangle_drivable_area = list_rectangles_drivable_area[idx_drivable_area]

        reach_node = construct_reach_node(rectangle_drivable_area, list_propagated_set,
                                          list_idx_propagated_sets_adjacent)
        if reach_node:
            reachable_set.append(reach_node)

    return reachable_set


def construct_reach_node(rectangle_drivable_area: ReachPolygon,
                         list_propagated_set: List[SemanticReachNode],
                         list_idx_propagated_sets_adjacent: List[int]) -> Optional[SemanticReachNode]:
    """
    Returns a reach node constructed from the propagated sets.

    Iterate through propagated sets that are adjacent to the drivable areas, and intersect the propagated sets with
    position constraints from the drivable areas. A non-empty intersected polygon imply that it is a valid base set and
    is considered as a parent of the rectangle (reachable from the node from which the propagated set was propagated).
    """
    list_nodes_parent = []
    list_vertices_polygon_lon_new = []
    list_vertices_polygon_lat_new = []
    # retrieve each of the adjacent propagated sets
    for idx_propagated_set_adjacent in list_idx_propagated_sets_adjacent:
        propagated_set_adjacent = list_propagated_set[idx_propagated_set_adjacent]
        polygon_lon = propagated_set_adjacent.polygon_lon
        polygon_lat = propagated_set_adjacent.polygon_lat
        # cut down to position range of the drivable area rectangle
        try:
            polygon_lon = polygon_lon.intersect_halfspace(1, 0, rectangle_drivable_area.p_lon_max)
            polygon_lon = polygon_lon.intersect_halfspace(-1, 0, -rectangle_drivable_area.p_lon_min)
            polygon_lat = polygon_lat.intersect_halfspace(1, 0, rectangle_drivable_area.p_lat_max)
            polygon_lat = polygon_lat.intersect_halfspace(-1, 0, -rectangle_drivable_area.p_lat_min)

        except AttributeError:
            pass

        else:
            # add to list if the intersected polygons are non-empty
            if polygon_lon and not polygon_lon.is_empty and polygon_lat and not polygon_lat.is_empty:
                list_vertices_polygon_lon_new += polygon_lon.vertices
                list_vertices_polygon_lat_new += polygon_lat.vertices
                node_parent = propagated_set_adjacent.source_propagation
                list_nodes_parent.append(node_parent)

    # if there is at least one valid propagated set, create node
    if list_vertices_polygon_lon_new and list_vertices_polygon_lat_new:
        polygon_lon_new = ReachPolygon.from_polygon(ReachPolygon(list_vertices_polygon_lon_new).convex_hull)
        polygon_lat_new = ReachPolygon.from_polygon(ReachPolygon(list_vertices_polygon_lat_new).convex_hull)
        proposition_holder = list_propagated_set[0].proposition_holder.clone()
        reach_node = SemanticReachNode(polygon_lon_new, polygon_lat_new, proposition_holder=proposition_holder)
        reach_node.source_propagation = list_nodes_parent

        return reach_node

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


def determine_connected_reach_nodes(list_nodes_reach: Union[List[SemanticReachNode], List[pycrreachs.SemanticReachNode]]):
    """
    Determines connected sets in the position domain.

    Returns a dictionary in the form of {node index self:list of tuples (node index self, node index other)}.
    This function is the equivalent python function to pycrreach.connected_reachset_boost().
    """
    dict_adjacency = defaultdict(list)

    if not list_nodes_reach:
        return dict_adjacency

    else:
        if isinstance(list_nodes_reach[0], SemanticReachNode):
            list_position_rectangles = [node_reach.position_rectangle for node_reach in list_nodes_reach]

        else:
            list_position_rectangles = [node_reach.position_rectangle() for node_reach in list_nodes_reach]

    # iterate over all rectangles
    for idx1, position_rect_1 in enumerate(list_position_rectangles):
        for idx2, position_rect_2 in enumerate(list_position_rectangles):
            if idx1 == idx2:
                continue

            if position_rect_1.intersects(position_rect_2):
                dict_adjacency[idx1].append((idx1, idx2))

    return dict_adjacency
