from collections import defaultdict
from typing import Dict, List, Tuple

from shapely.errors import TopologicalError
from shapely.geometry import Polygon, MultiPolygon

from commonroad_reach.data_structure.reach.reach_polygon import ReachPolygon
import commonroad_reach.utility.general as util_general

from commonroad_reach_semantic.data_structure.environment_model.region import Region
import commonroad_reach_semantic.utility.geometry as util_geometry


def detect_intersecting_lanelets(set_lanelets_related):
    """
    Returns a set of tuples of ids of lanelets that are intersecting.

    Note that the intersection property is propagated to find clusters of intersecting lanelets.
    """
    set_tuples_ids_lanelets_intersecting = set()
    # Create a default dictionary where each key is a lanelet ID and the value is a set of intersecting lanelet IDs
    dict_id_lanelet_to_set_ids_intersecting = defaultdict(set)

    for lanelet_i in set_lanelets_related:
        id_lanelet_i = lanelet_i.lanelet_id
        set_ids_lanelet_intersecting = dict_id_lanelet_to_set_ids_intersecting[id_lanelet_i]
        set_ids_lanelet_intersecting.add(id_lanelet_i)

        for lanelet_j in set_lanelets_related:
            id_lanelet_j = lanelet_j.lanelet_id

            # cases where we skip detecting intersection
            if id_lanelet_j == id_lanelet_i or id_lanelet_j in set_ids_lanelet_intersecting:
                continue

            if lanelet_i.polygon_cart.intersects(lanelet_j.polygon_cart):
                set_ids_lanelet_intersecting.add(id_lanelet_j)

        for lanelet_j in set_ids_lanelet_intersecting:
            dict_id_lanelet_to_set_ids_intersecting[lanelet_j].update(set_ids_lanelet_intersecting)

    for set_ids_intersecting in dict_id_lanelet_to_set_ids_intersecting.values():
        if len(set_ids_intersecting) > 1:
            set_tuples_ids_lanelets_intersecting.add(frozenset(set_ids_intersecting))
    # set_tuples_ids_lanelets_intersecting.add(frozenset({14, 19, 11, 13, 0, 5}))
    return set_tuples_ids_lanelets_intersecting


def construct_regions_for_intersecting_lanelets(set_tuples_ids_lanelets_intersecting):
    """
    Returns a list of regions for intersecting lanelets.
    """
    list_regions = []

    # iterate through all tuples of clusters of intersecting lanelets and create regions accordingly
    for set_ids_lanelets_in_cluster in set_tuples_ids_lanelets_intersecting:
        try:
            list_regions += construct_regions_from_tuple_ids_lanelets(set_ids_lanelets_in_cluster)
        except Exception as e:
            print(f"Error in constructing regions for intersecting lanelets: {e}")
            continue
    return list_regions


def construct_regions_from_tuple_ids_lanelets(set_ids_lanelets_in_cluster):
    """Constructs lists of regions for the given tuple of lanelet ids"""
    list_regions_output = []

    dict_cardinality_to_list_tuples_ids_lanelets: Dict[int, List] = defaultdict(list)
    dict_cardinality_to_list_regions: Dict[int, List[Region]] = defaultdict(list)
    dict_id_lanelet_to_polygon_cart_lanelet = dict()
    # add Cartesian polygons of lanelets to a dict
    for id_lanelet in set_ids_lanelets_in_cluster:
        dict_id_lanelet_to_polygon_cart_lanelet[id_lanelet] = \
            Region.lanelet_network.find_lanelet_by_id(id_lanelet).polygon_cart

    # create a dictionary which maps cardinality of a set to its list of elements in the power set
    for tuple_ids_lanelets in util_general.power_set(set_ids_lanelets_in_cluster):
        dict_cardinality_to_list_tuples_ids_lanelets[len(tuple_ids_lanelets)].append(tuple_ids_lanelets)
    cardinality_max = max(dict_cardinality_to_list_tuples_ids_lanelets)

    # create regions from sets with the most elements to the least elements
    empty_intersections = []
    for cardinality in range(cardinality_max, 0, -1):
        # iterate through all tuples with the same cardinality
        list_tuples_ids_lanelets = dict_cardinality_to_list_tuples_ids_lanelets[cardinality]
        for tuple_ids_lanelets in list_tuples_ids_lanelets:
            # if we know that a subset of the lanelets to intersect is already empty, we can skip this tuple
            if any(empty.issubset(frozenset(tuple_ids_lanelets)) for empty in empty_intersections):
                continue
            # retrieve polygons of lanelets
            list_polygons_cart_lanelets = \
                [dict_id_lanelet_to_polygon_cart_lanelet[id_lanelet] for id_lanelet in tuple_ids_lanelets]
            # first try to obtain the intersection of polygons of the given list of lanelets
            polygon_intersected, empty_idx = obtain_intersection_of_lanelet_polygons(list_polygons_cart_lanelets)
            if polygon_intersected.is_empty:
                empty_intersections.append(frozenset(tuple_ids_lanelets[:empty_idx + 1]))
                continue
            # then remove parts that also belong to the sets of lanelet ids with higher cardinality
            polygon_own, is_valid_polygon = obtain_own_polygon(polygon_intersected, tuple_ids_lanelets,
                                                                       dict_cardinality_to_list_regions,
                                                                       cardinality, cardinality_max)
            if not is_valid_polygon:
                continue
            # create a region for a valid polygon
            list_regions_new = create_regions_from_polygon(tuple_ids_lanelets, polygon_own)
            dict_cardinality_to_list_regions[cardinality] += list_regions_new

    for list_regions in dict_cardinality_to_list_regions.values():
        list_regions_output += list_regions

    return list_regions_output


def obtain_intersection_of_lanelet_polygons(list_polygons_cart_lanelets: List[Polygon]) -> Tuple[Polygon, int]:
    """Returns the intersection of the polygons of the given list of lanelets.

    Second return value is the index of the first polygon that made the intersection empty or -1 if no such polygon exists.
    """
    polygon_intersected = list_polygons_cart_lanelets[0]
    for i, polygon in enumerate(list_polygons_cart_lanelets[1:]):
        polygon_intersected: Polygon = polygon_intersected.intersection(polygon)
        # polygon_intersected = polygon_intersected.intersection(polygon).buffer(-0.01)

        if polygon_intersected.is_empty:
            return polygon_intersected, i + 1

    return polygon_intersected, -1


def obtain_own_polygon(polygon_intersected, tuple_ids_lanelets,
                       dict_cardinality_to_list_regions, cardinality, cardinality_max):
    """Returns the polygon that solely belong to the tuple of lanelet ids

    Takes the difference with the polygons of greater cardinality:
    e.g., polygon of {1, 2} should not intersect with polygons of {1, 2, 3} and {1, 2, 4}
    """
    is_valid_polygon = True
    for cardinality_greater in range(cardinality_max, cardinality, -1):
        for region_greater in dict_cardinality_to_list_regions[cardinality_greater]:
            # only take the difference if the current set is a subset of the region
            if not set(tuple_ids_lanelets).issubset(set(region_greater.set_ids_lanelets)):
                continue

            try:
                polygon_intersected = (polygon_intersected - region_greater.polygon_cart.shapely_object)

            except TopologicalError:
                is_valid_polygon = False
                break

            if polygon_intersected.is_empty:
                is_valid_polygon = False
                break

        if not is_valid_polygon:
            break

    if is_valid_polygon:
        polygon_intersected = polygon_intersected.buffer(Region.buffer_polygon)
        is_valid_polygon = not polygon_intersected.is_empty

    return polygon_intersected, is_valid_polygon


def create_regions_from_polygon(tuple_ids_lanelets, polygon_cart):
    """Creates a list of regions from the given polygon"""
    list_regions = []

    if isinstance(polygon_cart, Polygon):
        region = create_region_from_polygon(tuple_ids_lanelets, polygon_cart)
        if region:
            list_regions.append(region)

    elif isinstance(polygon_cart, MultiPolygon):
        for p_cart in polygon_cart.geoms:
            region = create_region_from_polygon(tuple_ids_lanelets, p_cart)
            if region:
                list_regions.append(region)

    return list_regions


def create_region_from_polygon(tuple_ids_lanelets, polygon_cart_shapely):
    """Creates a region from the given Cartesian polygon.

    A region is created only if it can be converted into one under curvilinear coordinate system.
    """
    if Region.discard_region_small and polygon_cart_shapely.area < Region.area_polygon_desired_min:
        return None

    # fixing potential issues in the shapely polygon
    polygon_cart_shapely.buffer(-0.2)
    polygon_cart_shapely.buffer(0.2)
    list_vertices_cart = [(x, y) for x, y in zip(polygon_cart_shapely.exterior.coords.xy[0],
                                                 polygon_cart_shapely.exterior.coords.xy[1])]
    region = None

    polygon_cvln = util_geometry.convert_to_curvilinear_polygon(polygon_cart_shapely,
                                                                Region.distance_vertices_polygon_max,
                                                                Region.CLCS)
    if polygon_cvln:
        region = Region(tuple_ids_lanelets, ReachPolygon(list_vertices_cart), "CART")
        region.assign_polygon(polygon_cvln, "CVLN")

    return region


def construct_regions_for_nonintersecting_lanelets(set_tuples_ids_lanelets_intersecting):
    """
    Returns a list of regions for nonintersecting lanelets.
    """
    list_regions = []

    # obtain the set of lanelet ids that are intersecting with other lanelets
    set_ids_lanelets_intersecting = set()
    for tuple_ids in set_tuples_ids_lanelets_intersecting:
        for id_lanelet in tuple_ids:
            set_ids_lanelets_intersecting.add(id_lanelet)

    for lanelet in Region.lanelet_network.lanelets:
        # skip the lanelet if it is within the set of intersecting lanelets
        if lanelet.lanelet_id in set_ids_lanelets_intersecting:
            continue

        # create a region for the lanelet that is not intersecting with others
        region = create_region_from_polygon([lanelet.lanelet_id], lanelet.polygon.shapely_object)
        if region:
            list_regions.append(region)

    return list_regions
