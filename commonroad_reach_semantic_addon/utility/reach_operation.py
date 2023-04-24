import logging
from typing import List, Optional

from commonroad.scenario.lanelet import Lanelet
from commonroad_reach.data_structure.reach.reach_node import ReachNode
from commonroad_reach.data_structure.reach.reach_polygon import ReachPolygon
from commonroad_reach.utility import geometry as util_geometry
from commonroad_reach.utility import logger as util_logger

from commonroad_reach_semantic_addon.data_structure.position_interval import PositionInterval
from commonroad_reach_semantic_addon.data_structure.proposition import PropositionGroup as PropGroup
from commonroad_reach_semantic_addon.data_structure.reach.semantic_reach_node import SemanticReachNode

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
        # all identified adjacencies should have the same propositions,
        # as we did not merge drivable areas with different propositions
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
