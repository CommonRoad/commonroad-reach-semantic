import numpy as np

from commonroad_reach.data_structure.reach.reach_polygon import ReachPolygon


def convert_to_curvilinear_polygon(polygon_cart_shapely, distance_vertices_max, CLCS):
    """
    Converts a polygon into curvilinear coordinate system.
    """
    polygon_cvln = None
    list_vertices_cvln = []

    list_vertices_cart = [(x, y) for x, y in zip(polygon_cart_shapely.exterior.coords.xy[0],
                                                 polygon_cart_shapely.exterior.coords.xy[1])]
    list_vertices_cart = interpolate_vertices(list_vertices_cart, distance_vertices_max)
    # convert the vertices into curvilinear coordinate system
    for x, y in list_vertices_cart:
        try:
            p_lon, p_lat = CLCS.convert_to_curvilinear_coords(x, y)

        except ValueError:
            continue

        else:
            list_vertices_cvln.append((p_lon, p_lat))

    # construct curvilinear polygon if there are enough vertices
    if len(list_vertices_cvln) >= 3:
        polygon_cvln = ReachPolygon(list_vertices_cvln)

    return polygon_cvln


def interpolate_vertices(list_vertices_cart, max_distance_vertices):
    """
    Interpolates vertices such that the distance between consecutive ones do not exceed the specified distance.
    """
    list_vertices_interpolated = []

    for vertex_current, vertex_next in zip(list_vertices_cart[:-1], list_vertices_cart[1:]):
        # append original vertex
        list_vertices_interpolated.append(vertex_current)

        # if the distance between current and next exceeds maximum distance, interpolate new one in between
        x_diff = vertex_next[0] - vertex_current[0]
        y_diff = vertex_next[1] - vertex_current[1]
        dis = np.linalg.norm(np.array([x_diff, y_diff]))
        num_vertices_interpolate = int(dis // max_distance_vertices)
        for index_vertex_interpolate in range(num_vertices_interpolate):
            ratio = (index_vertex_interpolate + 1) / (num_vertices_interpolate + 1)
            x_vertex = vertex_current[0] + x_diff * ratio
            y_vertex = vertex_current[1] + y_diff * ratio

            list_vertices_interpolated.append((x_vertex, y_vertex))

    # shapely polygon should be closed
    list_vertices_interpolated.append(list_vertices_cart[-1])

    return list_vertices_interpolated


def interpolate_vertex(p1: np.ndarray, p2: np.ndarray, length: float):
    # interpolate a point, with distance of 3 meters
    diff = p1 - p2
    dist = np.linalg.norm(diff)
    p0 = p1 + diff / dist * length

    return p0
