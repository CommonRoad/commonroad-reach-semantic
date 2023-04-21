from commonroad.scenario.lanelet import Lanelet


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