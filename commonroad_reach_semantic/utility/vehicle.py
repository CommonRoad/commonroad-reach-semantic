from typing import Union, Dict, Set, Optional, List

from commonroad.scenario.intersection import IntersectionIncomingElement
from commonroad.scenario.lanelet import Lanelet, LaneletType, LaneletNetwork
from commonroad.scenario.obstacle import StaticObstacle, DynamicObstacle
from commonroad.common.util import Interval, AngleInterval
from commonroad.geometry.shape import Rectangle
from commonroad.scenario.state import State, CustomState, InitialState
from commonroad.planning.goal import GoalRegion
from commonroad.planning.planning_problem import PlanningProblem

from commonroad_reach.data_structure.configuration import Configuration
from commonroad_route_planner.route_planner import RoutePlanner

from commonroad_reach_semantic.data_structure.config.outgoing_direction import OutgoingDirection
from commonroad_reach_semantic.data_structure.environment_model.road_network import RoadNetwork, Lane


def compute_acceleration(vel_pre: float, vel_cur: float, dt: float):
    """
    Computes acceleration given velocity

    :param vel_cur: velocity of current time step
    :param vel_pre: velocity of previous time step
    :param dt: time step size
    :return: acceleration
    """
    acc = (vel_cur - vel_pre) / dt
    return acc


def compute_jerk(current_acceleration: float, previous_acceleration: float, dt: float) -> float:
    """
    Computes jerk given acceleration

    :param current_acceleration: acceleration of current time step
    :param previous_acceleration: acceleration of previous time step
    :param dt: time step size
    :return: jerk
    """
    jerk = (current_acceleration - previous_acceleration) / dt
    return jerk


def find_main_carriage_way_lanelet_id(lanelet: Lanelet, lanelet_network) -> int:
    """
    Searches for an adjacent lanelet part of the main carriageway

    :param lanelet_network:
    :param lanelet: start lanelet
    :return: ID of a lanelet which is part of the main carriageway
    """
    current_lanelet = lanelet
    if LaneletType.MAIN_CARRIAGE_WAY in current_lanelet.lanelet_type:
        return current_lanelet.lanelet_id
    while current_lanelet.adj_left_same_direction is not None:
        current_lanelet = lanelet_network.find_lanelet_by_id(current_lanelet.adj_left)
        if LaneletType.MAIN_CARRIAGE_WAY in current_lanelet.lanelet_type:
            return current_lanelet.lanelet_id


def extract_sonia_extrema(dict_step_to_spot_prediction_occupancy: Dict, config: Configuration):
    """Returns the extrema of the given sonia prediction

    todo: compute extrema for polygons in different lanelets?
    """
    dict_step_to_sonia_extrema = {}

    # obtain extrema of sonia prediction polygons
    for time_step, polygon in dict_step_to_spot_prediction_occupancy.items():
        list_p_lon = []
        list_p_lat = []

        if not polygon:
            continue

        for shape in polygon.shape.shapes:
            for x, y in shape.vertices:
                try:
                    p_lon, p_lat = config.planning.CLCS.convert_to_curvilinear_coords(x, y)

                except ValueError:
                    continue

                else:
                    list_p_lon.append(p_lon)
                    list_p_lat.append(p_lat)

            p_lon_min = min(list_p_lon) if len(list_p_lon) else None
            p_lon_max = max(list_p_lon) if len(list_p_lon) else None
            p_lat_min = min(list_p_lat) if len(list_p_lat) else None
            p_lat_max = max(list_p_lat) if len(list_p_lat) else None
            dict_step_to_sonia_extrema[time_step] = (p_lon_min, p_lat_min, p_lon_max, p_lat_max)

    return dict_step_to_sonia_extrema


def determine_intersection_attributes(lanelet_network: LaneletNetwork, set_ids_lanelets):
    """Determines the intersection-related attributes of the vehicle."""
    incoming_element = None
    set_ids_lanelets_outgoing_left = set()
    set_ids_lanelets_outgoing_straight = set()
    set_ids_lanelets_outgoing_right = set()

    list_incomings = []
    dict_incoming_to_intersection = dict()
    for intersection in lanelet_network.intersections:
        list_incomings += intersection.incomings
        for incoming in intersection.incomings:
            dict_incoming_to_intersection[incoming] = intersection

    # determine incoming element
    for incoming in list_incomings:
        if incoming.incoming_lanelets.intersection(set_ids_lanelets):
            incoming_element = incoming
            break

    if incoming_element:
        # determine outgoings
        for id_lanelet in incoming_element.successors_left:
            lanelet = lanelet_network.find_lanelet_by_id(id_lanelet)
            if lanelet and lanelet.successor:
                set_ids_lanelets_outgoing_left.update(set(lanelet.successor))

        for id_lanelet in incoming_element.successors_straight:
            lanelet = lanelet_network.find_lanelet_by_id(id_lanelet)
            if lanelet and lanelet.successor:
                set_ids_lanelets_outgoing_straight.update(set(lanelet.successor))

        for id_lanelet in incoming_element.successors_right:
            lanelet = lanelet_network.find_lanelet_by_id(id_lanelet)
            if lanelet and lanelet.successor:
                set_ids_lanelets_outgoing_right.update(set(lanelet.successor))

    return incoming_element, set_ids_lanelets_outgoing_left, \
           set_ids_lanelets_outgoing_straight, set_ids_lanelets_outgoing_right


def extract_lane_of_vehicle(obstacle: Union[DynamicObstacle, StaticObstacle], road_network: RoadNetwork) -> Optional[Lane]:
    """Extracts the lane of the vehicle based on vehicle-specific future information.

    Only keeps one lane.
    """
    # forward restriction of lanes
    dict_step_to_set_lanes = {}
    set_lanes_initial = set()
    list_states_obstacle = [obstacle.initial_state]
    if isinstance(obstacle, DynamicObstacle):
        list_states_obstacle += obstacle.prediction.trajectory.state_list

    for state_cr in list_states_obstacle:
        step = state_cr.time_step
        [list_ids_lanelets] = road_network.lanelet_network.find_lanelet_by_position([state_cr.position])
        set_lanes_at_step = road_network.find_lanes_by_lanelets(list_ids_lanelets)

        # find the first nonempty set of lanes
        if not set_lanes_initial and set_lanes_at_step:
            set_lanes_initial.update(set_lanes_at_step)
        # we only allow keeping the lanes computed at the initial step
        dict_step_to_set_lanes[step] = set_lanes_initial.intersection(set_lanes_at_step)

    # backward extraction of lanes
    step_final = max(dict_step_to_set_lanes)
    set_lanes_last_nonempty = set()
    # find the last nonempty set of lanes
    for step in reversed(range(step_final + 1)):
        if step in dict_step_to_set_lanes.keys():
            if dict_step_to_set_lanes[step]:
                set_lanes_last_nonempty.update(dict_step_to_set_lanes[step])
                break

    # use anyone if there are multiple lanes
    if set_lanes_last_nonempty:
        return set_lanes_last_nonempty.pop()
    else:
        return None


def extract_incoming_from_lane(lane: Lane, lanelet_network: LaneletNetwork):
    """Extracts incoming element and the type of outgoing from the given lane."""
    id_lanelet_incoming = None
    id_lanelet_successor = None
    incoming_element = None
    direction_outgoing = None

    # examine whether the lane passes through an intersection
    for id_lanelet_pre, id_lanelet_suc in zip(lane.list_ids_lanelets[:-1], lane.list_ids_lanelets[1:]):
        lanelet_pre = lanelet_network.find_lanelet_by_id(id_lanelet_pre)
        lanelet_suc = lanelet_network.find_lanelet_by_id(id_lanelet_suc)

        # found two consecutive lanelets of type intersection
        if LaneletType.INTERSECTION in lanelet_pre.lanelet_type and \
                LaneletType.INTERSECTION in lanelet_suc.lanelet_type:
            id_lanelet_incoming = id_lanelet_pre
            id_lanelet_successor = id_lanelet_suc
            break

    if id_lanelet_incoming and id_lanelet_successor:
        # determine the incoming element of the lane
        for intersection in lanelet_network.intersections:
            for incoming in intersection.incomings:
                if id_lanelet_incoming in incoming.incoming_lanelets:
                    incoming_element = incoming

    if incoming_element:
        if id_lanelet_successor in incoming_element.successors_left:
            direction_outgoing = OutgoingDirection.LEFT

        elif id_lanelet_successor in incoming_element.successors_straight:
            direction_outgoing = OutgoingDirection.STRAIGHT

        elif id_lanelet_successor in incoming_element.successors_right:
            direction_outgoing = OutgoingDirection.RIGHT

    return incoming_element, direction_outgoing


def extract_outgoings_from_incoming(incoming_element: IntersectionIncomingElement, lanelet_network: LaneletNetwork):
    """Extracts outgoing lanelets from the incoming element."""
    if incoming_element is None:
        return set(), set(), set()

    set_ids_lanelets_outgoing_left = set()
    set_ids_lanelets_outgoing_straight = set()
    set_ids_lanelets_outgoing_right = set()

    # update outgoing lanelets
    for id_lanelet in incoming_element.successors_left:
        lanelet = lanelet_network.find_lanelet_by_id(id_lanelet)
        if lanelet and lanelet.successor:
            set_ids_lanelets_outgoing_left.update(set(lanelet.successor))

    for id_lanelet in incoming_element.successors_straight:
        lanelet = lanelet_network.find_lanelet_by_id(id_lanelet)
        if lanelet and lanelet.successor:
            set_ids_lanelets_outgoing_straight.update(set(lanelet.successor))

    for id_lanelet in incoming_element.successors_right:
        lanelet = lanelet_network.find_lanelet_by_id(id_lanelet)
        if lanelet and lanelet.successor:
            set_ids_lanelets_outgoing_right.update(set(lanelet.successor))

    return set_ids_lanelets_outgoing_left, set_ids_lanelets_outgoing_straight, set_ids_lanelets_outgoing_right


def extract_oncomings_from_incoming(incoming_element: IntersectionIncomingElement,
                                    lanelet_network: LaneletNetwork) -> Set[int]:
    """Extracts oncoming lanelet ids for the given incoming element."""
    # collect all lanelets including incomings/straight successors/straight outgoings lanelets
    if incoming_element is None:
        return set()

    _, set_ids_lanelets_outgoing_straight, _ = extract_outgoings_from_incoming(incoming_element, lanelet_network)
    set_ids_lanelets_straight = incoming_element.incoming_lanelets.union(incoming_element.successors_straight,
                                                                         set_ids_lanelets_outgoing_straight)

    # find lanelets which have the opposite driving direction as the collected lanelets
    set_ids_lanelets_oncoming = set()
    for id_lanelet in set_ids_lanelets_straight:
        lanelet = lanelet_network.find_lanelet_by_id(id_lanelet)
        if lanelet.adj_left and not lanelet.adj_left_same_direction:
            set_ids_lanelets_oncoming.add(lanelet.adj_left)

    # find all remaining lanelets that have the opposite driving directions as the collected lanelets
    completed = False
    while not completed:
        set_ids_lanelets_to_be_added = set()
        for id_lanelet_oncoming in set_ids_lanelets_oncoming:
            lanelet_oncoming = lanelet_network.find_lanelet_by_id(id_lanelet_oncoming)
            if lanelet_oncoming.adj_right and lanelet_oncoming.adj_right_same_direction and \
                    lanelet_oncoming.adj_right not in set_ids_lanelets_to_be_added:
                set_ids_lanelets_to_be_added.add(lanelet_oncoming.adj_right)
        set_ids_lanelets_oncoming.update(set_ids_lanelets_to_be_added)

        completed = not len(set_ids_lanelets_to_be_added)

    return set_ids_lanelets_oncoming

def initialize_lanelets_dir(lanelet_network: LaneletNetwork,
                            obstacle_states: List[Union[CustomState, State]]):
    """Initializes the direction of lanelets."""
    ini_state = obstacle_states[0]
    end_state = obstacle_states[-1]

    attributes = {
        "time_step": Interval(start=end_state.time_step - 1, end=end_state.time_step + 1),
        "position": Rectangle(length=1.0, width=1.0, center=end_state.position),
        "velocity": Interval(start=end_state.velocity, end=end_state.velocity + 1),
        "orientation": AngleInterval(
            start=end_state.orientation - 0.1, end=end_state.orientation + 0.1
        ),
    }
    route = _find_route_given_initial_goal(ini_state, attributes, lanelet_network)
    return route.lanelet_ids


def _find_route_given_initial_goal(initial_state: Union[CustomState, State],
                                   goal_attribute: Optional[Dict],
                                   lanelet_network: LaneletNetwork):
    """
    Finds a route given the initial state and goal attribute.
    """
    planning_problem = PlanningProblem(0,
                                       InitialState(
                                           position=initial_state.position,
                                           velocity=initial_state.velocity,
                                           orientation=initial_state.orientation,
                                           yaw_rate=0.,
                                           slip_angle=0.,
                                           time_step=initial_state.time_step
                                       ),
                                       GoalRegion(state_list=[
                                           CustomState(**goal_attribute)
                                       ]))
    route_planner = RoutePlanner(
        lanelet_network=lanelet_network,
        planning_problem=planning_problem
    )
    candidate_holder = route_planner.plan_routes()
    route = candidate_holder.retrieve_shortetest_route_with_least_lane_changes()
    return route
