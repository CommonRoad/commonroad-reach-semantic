import enum
from collections import defaultdict
from typing import Union, Dict, List, Set, Optional

import numpy as np
# from commonroad_reach_semantic_addon import pycrreachs
from commonroad.geometry.shape import Shape, Rectangle
from commonroad.scenario.intersection import IntersectionIncomingElement
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.obstacle import ObstacleType, SignalState, StaticObstacle, DynamicObstacle
from commonroad.scenario.traffic_sign import TrafficSign, TrafficLight
from commonroad.scenario.trajectory import State
from commonroad_dc.pycrccosy import CurvilinearCoordinateSystem
from commonroad_route_planner.route import Route

from commonroad_reach_semantic_addon.data_structure.semantic_configuration import SemanticConfiguration
from commonroad_reach_semantic_addon.data_structure.reach.semantic_reach_node import SemanticReachNode
from commonroad_reach_semantic_addon.data_structure.road_network import Lane, RoadNetwork
import commonroad_reach.utility.coordinate_system as util_cosy
import commonroad_reach_semantic_addon.utility.vehicle as util_vehicle


class StateLongitudinal:
    """
    Longitudinal state in curvilinear coordinate system.
    """
    __slots__ = ['s', 'v', 'a', 'j']

    def __init__(self, **kwargs):
        """
        Elements of state vector are determined during runtime.
        """
        for (field, value) in kwargs.items():
            setattr(self, field, value)

    @property
    def attributes(self) -> List[str]:
        """
        Returns all dynamically set attributes of an instance of State.

        :return: subset of slots which are dynamically assigned to the object.
        """
        attributes = list()
        for slot in self.__slots__:
            if hasattr(self, slot):
                attributes.append(slot)
        return attributes

    def __repr__(self):
        state = ''
        for attr in self.attributes:
            state += attr
            state += '= {} '.format(self.__getattribute__(attr))
        return state


class StateLateral:
    """
    Lateral state in curvilinear coordinate system.
    """
    __slots__ = ['d', 'theta', 'kappa', 'kappa_dot']

    def __init__(self, **kwargs):
        """
        Elements of state vector are determined during runtime.
        """
        for (field, value) in kwargs.items():
            setattr(self, field, value)

    @property
    def attributes(self) -> List[str]:
        """
        Returns all dynamically set attributes of an instance of State.

        :return: subset of slots which are dynamically assigned to the object.
        """
        attributes = list()
        for slot in self.__slots__:
            if hasattr(self, slot):
                attributes.append(slot)
        return attributes

    def __repr__(self):
        state = '\n'
        for attr in self.attributes:
            state += attr
            state += '= {}\n'.format(self.__getattribute__(attr))
        return state


@enum.unique
class VehicleClassification(enum.Enum):
    EGO_VEHICLE = 0
    UNDECIDED = 1
    CROSSING_VEHICLE = 2
    ADJACENT_VEHICLE = 3


class Vehicle:
    """
    Class to represent vehicle objects.

    Ref: attributes projected onto the reference path of the planned route
    Ego: attributes projected onto the reference path of the lane of the vehicle
    """
    config: SemanticConfiguration
    CLCS_ref: CurvilinearCoordinateSystem
    lanelet_network: LaneletNetwork
    road_network: RoadNetwork
    radius_inflation: float

    @classmethod
    def initialize(cls, config: SemanticConfiguration, road_network: RoadNetwork):
        cls.config = config
        cls.CLCS_ref = config.planning.CLCS
        cls.lanelet_network = config.scenario.lanelet_network
        cls.road_network = road_network
        cls.radius_inflation = config.vehicle.ego.radius_inflation

    def __init__(self,
                 lane: Lane,
                 dict_step_to_state_cr: Dict[int, State],
                 dict_step_to_state_lon_ref: Dict[int, StateLongitudinal],
                 dict_step_to_state_lat_ref: Dict[int, StateLateral],
                 dict_step_to_state_lon_ego: Dict[int, StateLongitudinal],
                 dict_step_to_state_lat_ego: Dict[int, StateLateral],
                 shape: Union[Shape, Rectangle],
                 vehicle_id: int,
                 obstacle_type: ObstacleType,
                 dict_step_to_list_ids_lanelets: Dict[int, List[int]],
                 incoming_element: Optional[IntersectionIncomingElement],
                 direction_outgoing: str,
                 set_ids_lanelets_outgoing_left: Set[int],
                 set_ids_lanelets_outgoing_straight: Set[int],
                 set_ids_lanelets_outgoing_right: Set[int],
                 set_ids_lanelets_oncoming: Set[int],
                 dict_step_to_state_signal: Dict[int, SignalState] = None,
                 use_sonia: bool = False,
                 dict_step_to_sonia_prediction_occupancy: Dict = None,
                 dict_step_to_sonia_extrema: Dict = None):
        self.lane = lane
        self.shape = shape
        self.id_vehicle = vehicle_id
        self.type_obstacle = obstacle_type
        self.dict_step_to_state_cr = dict_step_to_state_cr
        self.dict_step_to_state_lon_ref = dict_step_to_state_lon_ref
        self.dict_step_to_state_lat_ref = dict_step_to_state_lat_ref
        self.dict_step_to_state_lon_ego = dict_step_to_state_lon_ego
        self.dict_step_to_state_lat_ego = dict_step_to_state_lat_ego
        self.dict_step_to_state_signal = dict_step_to_state_signal
        self.dict_step_to_list_ids_lanelets = dict_step_to_list_ids_lanelets

        # intersection-related
        self.incoming_element = incoming_element
        self.type_outgoing = direction_outgoing
        self.set_ids_lanelets_outgoing_left = set_ids_lanelets_outgoing_left
        self.set_ids_lanelets_outgoing_straight = set_ids_lanelets_outgoing_straight
        self.set_ids_lanelets_outgoing_right = set_ids_lanelets_outgoing_right
        self.set_ids_lanelets_oncoming = set_ids_lanelets_oncoming
        self.dict_step_to_priorities = defaultdict()
        self.dict_id_lanelet_to_priorities = defaultdict(dict)

        # sonia-related
        self.use_sonia = use_sonia
        self.dict_step_to_sonia_prediction_occupancy = dict_step_to_sonia_prediction_occupancy
        self.dict_step_to_sonia_extrema = dict_step_to_sonia_extrema

        self._update_traffic_lights_and_signs()

    def __repr__(self):
        return f"Vehicle(id={self.id_vehicle})"

    def _update_traffic_lights_and_signs(self):
        """
        Completes attributes from other attributes.
        """
        self.dict_step_to_set_traffic_signs = defaultdict(set)
        self.dict_step_to_set_traffic_lights = defaultdict(set)

        for step, list_ids_lanelets in self.dict_step_to_list_ids_lanelets.items():
            # only consider lanelets that are on the lane
            list_ids_lanelets_valid = set(self.lane.list_ids_lanelets).intersection(set(list_ids_lanelets))

            for id_lanelet in list_ids_lanelets_valid:
                lanelet = self.lanelet_network.find_lanelet_by_id(id_lanelet)

                for id_traffic_sign in lanelet.traffic_signs:
                    traffic_sign = self.lanelet_network.find_traffic_sign_by_id(id_traffic_sign)
                    self.dict_step_to_set_traffic_signs[step].add(traffic_sign)

                for id_traffic_light in lanelet.traffic_lights:
                    traffic_light = self.lanelet_network.find_traffic_light_by_id(id_traffic_light)
                    self.dict_step_to_set_traffic_lights[step].add(traffic_light)

    @property
    def list_states_cr(self) -> List[State]:
        list_state = [state for state in self.dict_step_to_state_cr.values()]
        return list_state

    @property
    def set_ids_lanelets_outgoing_lane(self) -> Set[int]:
        if self.type_outgoing in ["left", "straight", "right"]:
            return eval(f"self.set_ids_lanelets_outgoing_{self.type_outgoing}")

        else:
            return set()

    @property
    def set_ids_lanelets_successor_incoming(self) -> Set[int]:
        if not self.incoming_element:
            return set()

        else:
            set_ids_lanelets: Set[int] = eval(f"self.incoming_element.successors_{self.type_outgoing}")
            set_ids_lanelets_to_add = set()

            for id_lanelet in set_ids_lanelets:
                lanelet = self.lanelet_network.find_lanelet_by_id(id_lanelet)
                set_ids_lanelets_to_add.update(set(lanelet.successor))

            set_ids_lanelets.update(set_ids_lanelets_to_add)
            return set_ids_lanelets

    def spot_prediction_occupancy_at_step(self, step: int):
        return self.dict_step_to_sonia_prediction_occupancy[step]

    def lanelet_ids_at_step(self, step: int) -> List[int]:
        return self.dict_step_to_list_ids_lanelets.get(step, [])

    def traffic_signs_at_step(self, step: int) -> Set[TrafficSign]:
        return self.dict_step_to_set_traffic_signs.get(step, set())

    def traffic_lights_at_step(self, step: int) -> Set[TrafficLight]:
        return self.dict_step_to_set_traffic_lights.get(step, set())

    def incoming_at_step(self, step: int) -> Optional[IntersectionIncomingElement]:
        if set(self.lanelet_ids_at_step(step)).intersection(self.incoming_element.incoming_lanelets):
            return self.incoming_element

        else:
            return None

    def left_outgoings_at_step(self, step: int) -> Set[int]:
        if self.incoming_element:
            if set(self.lanelet_ids_at_step(step)).intersection(self.incoming_element.incoming_lanelets) or \
                    self.type_outgoing == "left":
                return self.set_ids_lanelets_outgoing_left

        return set()

    def straight_outgoings_at_step(self, step: int) -> Set[int]:
        if self.incoming_element:
            if set(self.lanelet_ids_at_step(step)).intersection(self.incoming_element.incoming_lanelets) or \
                    self.type_outgoing == "straight":
                return self.set_ids_lanelets_outgoing_straight

        return set()

    def right_outgoings_at_step(self, step: int) -> Set[int]:
        if self.incoming_element:
            if set(self.lanelet_ids_at_step(step)).intersection(self.incoming_element.incoming_lanelets) or \
                    self.type_outgoing == "right":
                return self.set_ids_lanelets_outgoing_right

        return set()

    def retrieve_p_lon_node_ego(self, node: Union[SemanticReachNode]):
        if isinstance(node, SemanticReachNode):
            bounds = node.position_rectangle.bounds

        else:
            bounds = node.position_rectangle().bounding_box()

        [polygon_cart] = util_cosy.convert_to_cartesian_polygon(bounds, self.CLCS_ref, False)
        list_vertices_cvln = util_cosy.convert_to_curvilinear_vertices(polygon_cart.vertices, self.lane.CLCS)
        list_p_lon_node = [vertex[0] for vertex in list_vertices_cvln]

        return list_p_lon_node

    def behind_node_at_step(self, step: int, node: SemanticReachNode) -> bool:
        """Returns True if vehicle is behind the base set."""
        return True if self.front_distance_to_node_at_step(step, node) > 0 else False

    def braking_caused_by_node_at_step(self, step: int, node: Union[SemanticReachNode]) -> bool:
        """
        Returns whether the vehicle should brake hard due to the reach node.
        """
        small_distance = self.front_distance_to_node_at_step(step, node) < \
                         self.config.traffic_rule.distance_braking
        brake_hard = self.should_brake_hard_due_to_node_at_step(step, node)

        return small_distance and brake_hard

    def front_distance_to_node_at_step(self, step: int, node: Union[SemanticReachNode]) -> float:
        """
        Returns the front distance of the vehicle to the base set.

        Compares p_min of the reach node and p_max of the vehicle.
        """
        try:
            p_lon_max_ego = self.p_lon_ego(step) + self.shape.length / 2

        except KeyError:
            return np.inf

        else:
            list_p_lon_node_ego = self.retrieve_p_lon_node_ego(node)
            if not list_p_lon_node_ego:
                return np.inf

            p_lon_min_node_ego = min(list_p_lon_node_ego) - self.radius_inflation
            return p_lon_min_node_ego - p_lon_max_ego

    def should_brake_hard_due_to_node_at_step(self, step: int, node: Union[SemanticReachNode]) -> bool:
        """
        Returns whether the vehicle should brake harder than the predefined threshold due to the reach node.
        """
        v_lon_ego = self.v_lon_ego(step)
        distance_front = self.front_distance_to_node_at_step(step, node)
        # if the reach node is not in front of the vehicle
        # if distance_front < 0: # this might be conservative at turnings
        if distance_front < self.config.vehicle.other.length / 2:
            return False

        distance_with_reaction = distance_front - v_lon_ego * self.config.vehicle.other.t_react
        # if the distance between the base set and the vehicle considering reaction is negative
        if distance_with_reaction < 0:
            return True

        # compute acceleration required to come to a full stop
        acc_brake = -v_lon_ego ** 2 / (2 * distance_with_reaction)
        if acc_brake < self.config.traffic_rule.acceleration_braking_hard:
            return True

        else:
            return False

    def rear_s_ref(self, step: int) -> float:
        """
        Calculates rear s-coordinate of vehicle

        :param step: time step to consider
        :returns rear s-coordinate [m]
        """
        s = self.dict_step_to_state_lon_ref[step].s
        width = self.shape.width
        length = self.shape.length
        theta = self.dict_step_to_state_lat_ref[step].theta

        return min((length / 2) * np.cos(theta) - (width / 2) * np.sin(theta) + s,
                   (length / 2) * np.cos(theta) - (-width / 2) * np.sin(theta) + s,
                   (-length / 2) * np.cos(theta) - (width / 2) * np.sin(theta) + s,
                   (-length / 2) * np.cos(theta) - (-width / 2) * np.sin(theta) + s)

    def front_s_ref(self, step: int) -> float:
        """
        Calculates front s-coordinate of vehicle

        :param step: time step to consider
        :returns front s-coordinate [m]
        """
        s = self.dict_step_to_state_lon_ref[step].s
        width = self.shape.width
        length = self.shape.length
        theta = self.dict_step_to_state_lat_ref[step].theta

        return max((length / 2) * np.cos(theta) - (width / 2) * np.sin(theta) + s,
                   (length / 2) * np.cos(theta) - (-width / 2) * np.sin(theta) + s,
                   (-length / 2) * np.cos(theta) - (width / 2) * np.sin(theta) + s,
                   (-length / 2) * np.cos(theta) - (-width / 2) * np.sin(theta) + s)

    def right_d_ref(self, step: int) -> float:
        """
        Calculates right d-coordinate of vehicle

        :param step: time step to consider
        :returns right d-coordinate [m]
        """
        d = self.dict_step_to_state_lat_ref[step].d
        width = self.shape.width
        length = self.shape.length
        theta = self.dict_step_to_state_lat_ref[step].theta

        return min((width / 2) * np.cos(theta) - (length / 2) * np.sin(theta) + d,
                   (width / 2) * np.cos(theta) - (-length / 2) * np.sin(theta) + d,
                   (-width / 2) * np.cos(theta) - (length / 2) * np.sin(theta) + d,
                   (-width / 2) * np.cos(theta) - (-length / 2) * np.sin(theta) + d)

    def left_d_ref(self, step: int) -> float:
        """
        Calculates left d-coordinate of vehicle

        :param step: step to consider
        :returns left d-coordinate [m]
        """
        d = self.dict_step_to_state_lat_ref[step].d
        width = self.shape.width
        length = self.shape.length
        theta = self.dict_step_to_state_lat_ref[step].theta

        return max((width / 2) * np.cos(theta) - (length / 2) * np.sin(theta) + d,
                   (width / 2) * np.cos(theta) - (-length / 2) * np.sin(theta) + d,
                   (-width / 2) * np.cos(theta) - (length / 2) * np.sin(theta) + d,
                   (-width / 2) * np.cos(theta) - (-length / 2) * np.sin(theta) + d)

    def p_lon_ref(self, step: int):
        return self.dict_step_to_state_lon_ref[step].s

    def p_lat_ref(self, step: int):
        return self.dict_step_to_state_lat_ref[step].d

    def v_lon_ref(self, step: int):
        return self.dict_step_to_state_lon_ref[step].v

    def p_lon_ego(self, step: int):
        return self.dict_step_to_state_lon_ego[step].s

    def p_lat_ego(self, step: int):
        return self.dict_step_to_state_lat_ego[step].d

    def v_lon_ego(self, step: int):
        return self.dict_step_to_state_lon_ego[step].v

    def p_lon_min_ref(self, step: int, half_length_ego: float = 0):
        if self.use_sonia:
            p_lon_min = self.dict_step_to_sonia_extrema[step][0]

        else:
            p_lon_min = self.rear_s_ref(step)

        return p_lon_min - half_length_ego

    def p_lon_max_ref(self, step: int, half_length_ego: float = 0):
        if self.use_sonia:
            p_lon_max = self.dict_step_to_sonia_extrema[step][2]

        else:
            p_lon_max = self.front_s_ref(step)

        return p_lon_max + half_length_ego

    def p_lat_min_ref(self, step: int, half_width_ego: float = 0):
        if self.use_sonia:
            p_lat_min = self.dict_step_to_sonia_extrema[step][1]

        else:
            p_lat_min = self.right_d_ref(step)

        return p_lat_min - half_width_ego

    def p_lat_max_ref(self, step: int, half_width_ego: float = 0):
        if self.use_sonia:
            p_lat_max = self.dict_step_to_sonia_extrema[step][3]

        else:
            p_lat_max = self.left_d_ref(step)

        return p_lat_max + half_width_ego

    def left_priority_at_step(self, step: int):
        priority_at_time = self.dict_step_to_priorities.get(step, None)
        return priority_at_time[0] if priority_at_time else None

    def straight_priority_at_step(self, step: int):
        priority_at_time = self.dict_step_to_priorities.get(step, None)
        return priority_at_time[1] if priority_at_time else None

    def right_priority_at_step(self, step: int):
        priority_at_time = self.dict_step_to_priorities.get(step, None)
        return priority_at_time[2] if priority_at_time else None

    def determine_priorities(self, dict_traffic_sign_to_priorities: Dict):
        """Determine traffic priorities based on the given dictionary."""
        priority_default = 3  # default priority from traffic sign 102 (right before left)
        # obtain priorities for lanelets of the vehicle
        for step, list_ids_lanelets in self.dict_step_to_list_ids_lanelets.items():
            for id_lanelet in list_ids_lanelets:
                index_min = np.inf
                element_index_min = None
                lanelet = self.lanelet_network.find_lanelet_by_id(id_lanelet)

                for id_traffic_sign in lanelet.traffic_signs:
                    traffic_sign = self.lanelet_network.find_traffic_sign_by_id(id_traffic_sign)

                    for element in traffic_sign.traffic_sign_elements:
                        try:
                            index = dict_traffic_sign_to_priorities[element.traffic_sign_element_id]["index"]

                        except KeyError:
                            index = 20

                        if index < index_min:
                            index_min = index
                            element_index_min = element

                try:
                    self.dict_id_lanelet_to_priorities[id_lanelet]["left"] = \
                        dict_traffic_sign_to_priorities[element_index_min.traffic_sign_element_id]["left"]
                    self.dict_id_lanelet_to_priorities[id_lanelet]["straight"] = \
                        dict_traffic_sign_to_priorities[element_index_min.traffic_sign_element_id]["straight"]
                    self.dict_id_lanelet_to_priorities[id_lanelet]["right"] = \
                        dict_traffic_sign_to_priorities[element_index_min.traffic_sign_element_id]["right"]

                except (KeyError, AttributeError):
                    self.dict_id_lanelet_to_priorities[id_lanelet]["left"] = priority_default
                    self.dict_id_lanelet_to_priorities[id_lanelet]["straight"] = priority_default
                    self.dict_id_lanelet_to_priorities[id_lanelet]["right"] = priority_default

        # obtain priorities for the vehicle
        for step, set_traffic_signs in self.dict_step_to_set_traffic_signs.items():
            index_min = np.inf
            element_index_min = None
            for traffic_sign in set_traffic_signs:
                for element in traffic_sign.traffic_sign_elements:
                    try:
                        index = dict_traffic_sign_to_priorities[element.traffic_sign_element_id]["index"]

                    except KeyError:
                        index = 20

                    if index < index_min:
                        index_min = index
                        element_index_min = element

            try:
                self.dict_step_to_priorities[step] = \
                    (dict_traffic_sign_to_priorities[element_index_min.traffic_sign_element_id]["left"],
                     dict_traffic_sign_to_priorities[element_index_min.traffic_sign_element_id]["straight"],
                     dict_traffic_sign_to_priorities[element_index_min.traffic_sign_element_id]["right"])

            except (KeyError, AttributeError):
                pass

    @classmethod
    def create_vehicle_from_obstacle(cls, obstacle: Union[StaticObstacle, DynamicObstacle],
                                     dict_sonia_prediction: Dict = dict) -> Optional['Vehicle']:
        """
        Transforms a CommonRoad obstacle into a Vehicle object.
        """
        use_sonia = cls.config.semantic_model.use_sonia and isinstance(obstacle, DynamicObstacle)
        dict_step_to_sonia_extrema = {}

        # extract properties for static obstacles
        if isinstance(obstacle, StaticObstacle):
            lane, dict_step_to_state_cr, \
                dict_step_to_state_lon_ref, dict_step_to_state_lat_ref, \
                dict_step_to_state_lon_ego, dict_step_to_state_lat_ego, \
                dict_step_to_state_signal, dict_step_to_list_ids_lanelets, \
                incoming_element, direction_outgoing, set_ids_lanelets_outgoing_left, \
                set_ids_lanelets_outgoing_straight, set_ids_lanelets_outgoing_right, set_ids_lanelets_oncoming, \
                dict_step_to_sonia_prediction_occupancy = \
                cls.extract_vehicle_attributes_from_static_obstacle(obstacle)

        # extract properties for dynamic obstacles
        elif isinstance(obstacle, DynamicObstacle):
            lane, dict_step_to_state_cr, \
                dict_step_to_state_lon_ref, dict_step_to_state_lat_ref, \
                dict_step_to_state_lon_ego, dict_step_to_state_lat_ego, \
                dict_step_to_state_signal, dict_step_to_list_ids_lanelets, \
                incoming_element, direction_outgoing, set_ids_lanelets_outgoing_left, \
                set_ids_lanelets_outgoing_straight, set_ids_lanelets_outgoing_right, set_ids_lanelets_oncoming, \
                dict_step_to_sonia_prediction_occupancy = \
                cls.extract_vehicle_attributes_from_dynamic_obstacle(obstacle, dict_sonia_prediction)
            if use_sonia:
                dict_step_to_sonia_extrema = \
                    util_vehicle.extract_sonia_extrema(dict_step_to_sonia_prediction_occupancy, cls.config)

        else:
            print(f"Obstacle with ID {obstacle.obstacle_id} is neither a static nor dynamic obstacle,"
                  f"skipping conversion to Vehicle.")

            return None

        # create vehicle object from extracted properties
        vehicle = Vehicle(lane, dict_step_to_state_cr, dict_step_to_state_lon_ref, dict_step_to_state_lat_ref,
                          dict_step_to_state_lon_ego, dict_step_to_state_lat_ego,
                          obstacle.obstacle_shape, obstacle.obstacle_id, obstacle.obstacle_type,
                          dict_step_to_list_ids_lanelets, incoming_element, direction_outgoing,
                          set_ids_lanelets_outgoing_left, set_ids_lanelets_outgoing_straight,
                          set_ids_lanelets_outgoing_right, set_ids_lanelets_oncoming,
                          dict_step_to_state_signal, use_sonia,
                          dict_step_to_sonia_prediction_occupancy, dict_step_to_sonia_extrema)

        return vehicle

    @classmethod
    def extract_vehicle_attributes_from_static_obstacle(cls, obstacle: StaticObstacle):
        # create empty dictionaries
        dict_step_to_state_cr = {}
        dict_step_to_state_lon_ref = {}
        dict_step_to_state_lat_ref = {}
        dict_step_to_state_lon_ego = {}
        dict_step_to_state_lat_ego = {}
        dict_step_to_state_signal = {}
        dict_step_to_list_ids_lanelets = {}
        dict_step_to_sonia_prediction_occupancy = {}

        dt = cls.config.planning.dt
        steps_computation = cls.config.planning.steps_computation

        lane = util_vehicle.extract_lane_of_vehicle(obstacle, cls.road_network)
        incoming_element, direction_outgoing = util_vehicle.extract_incoming_from_lane(lane, cls.lanelet_network)
        set_ids_lanelets_outgoing_left, \
            set_ids_lanelets_outgoing_straight, \
            set_ids_lanelets_outgoing_right = util_vehicle.extract_outgoings_from_incoming(incoming_element,
                                                                                           cls.lanelet_network)
        set_ids_lanelets_oncoming = util_vehicle.extract_oncomings_from_incoming(incoming_element, cls.lanelet_network)

        state_cr_init = obstacle.initial_state
        state_lon_ref, state_lat_ref, state_lon_ego, state_lat_ego = \
            cls.convert_to_curvilinear_state(state_cr_init, state_cr_init, dt,
                                             cls.config.planning.route, cls.CLCS_ref, lane, lane.CLCS)
        [list_ids_lanelets] = cls.lanelet_network.find_lanelet_by_position([state_cr_init.position])
        for step in range(steps_computation + 1):
            time_step = step * round(dt * 10)
            dict_step_to_state_cr[step] = state_cr_init
            dict_step_to_state_lon_ref[step] = state_lon_ref
            dict_step_to_state_lat_ref[step] = state_lat_ref
            dict_step_to_state_lon_ego[step] = state_lon_ego
            dict_step_to_state_lat_ego[step] = state_lat_ego
            dict_step_to_state_signal[step] = obstacle.signal_state_at_time_step(time_step)
            dict_step_to_list_ids_lanelets[step] = list_ids_lanelets

        return lane, dict_step_to_state_cr, \
            dict_step_to_state_lon_ref, dict_step_to_state_lat_ref, \
            dict_step_to_state_lon_ego, dict_step_to_state_lat_ego, \
            dict_step_to_state_signal, dict_step_to_list_ids_lanelets, \
            incoming_element, direction_outgoing, set_ids_lanelets_outgoing_left, \
            set_ids_lanelets_outgoing_straight, set_ids_lanelets_outgoing_right, set_ids_lanelets_oncoming, \
            dict_step_to_sonia_prediction_occupancy

    @classmethod
    def extract_vehicle_attributes_from_dynamic_obstacle(cls, obstacle: DynamicObstacle, dict_sonia_prediction: Dict):
        # create empty dictionaries
        dict_step_to_state_cr = {}
        dict_step_to_state_lon_ref = {}
        dict_step_to_state_lat_ref = {}
        dict_step_to_state_lon_ego = {}
        dict_step_to_state_lat_ego = {}
        dict_step_to_state_signal = {}
        dict_step_to_list_ids_lanelets = {}
        dict_step_to_sonia_prediction_occupancy = {}

        dt = cls.config.planning.dt
        dict_sonia_prediction = dict_sonia_prediction[obstacle.obstacle_id]

        lane_vehicle = util_vehicle.extract_lane_of_vehicle(obstacle, cls.road_network)
        # extract intersection-related attributes
        incoming_element, direction_outgoing = util_vehicle.extract_incoming_from_lane(lane_vehicle,
                                                                                       cls.lanelet_network)
        set_ids_lanelets_outgoing_left, \
            set_ids_lanelets_outgoing_straight, \
            set_ids_lanelets_outgoing_right = util_vehicle.extract_outgoings_from_incoming(incoming_element,
                                                                                           cls.lanelet_network)
        set_ids_lanelets_oncoming = util_vehicle.extract_oncomings_from_incoming(incoming_element, cls.lanelet_network)

        state_cr_previous = None
        list_states_obstacle_all = [obstacle.initial_state] + obstacle.prediction.trajectory.state_list
        # sample states based on specified dt
        list_states_obstacle_sampled = list_states_obstacle_all[::round(dt * 10)]

        for step, state_cr in enumerate(list_states_obstacle_sampled):
            if not state_cr_previous:
                state_cr_previous = state_cr

            state_lon_ref, state_lat_ref, state_lon_ego, state_lat_ego = \
                cls.convert_to_curvilinear_state(state_cr, state_cr_previous, dt,
                                                 cls.config.planning.route, cls.CLCS_ref, lane_vehicle,
                                                 lane_vehicle.CLCS)

            dict_step_to_state_cr[step] = state_cr
            dict_step_to_state_lon_ref[step] = state_lon_ref
            dict_step_to_state_lat_ref[step] = state_lat_ref
            dict_step_to_state_lon_ego[step] = state_lon_ego
            dict_step_to_state_lat_ego[step] = state_lat_ego
            dict_step_to_state_signal[step] = obstacle.signal_state_at_time_step(state_cr.time_step)
            [list_ids_lanelets] = cls.lanelet_network.find_lanelet_by_position([state_cr.position])
            dict_step_to_list_ids_lanelets[step] = list_ids_lanelets

            try:
                # occupancy_predicted is a list of tuples of (step, (step, polygon)
                dict_step_to_sonia_prediction_occupancy[step] = dict_sonia_prediction["occupancy_predicted"][step][1]

            except (IndexError, KeyError):
                dict_step_to_sonia_prediction_occupancy[step] = None

            state_cr_previous = state_cr

        return lane_vehicle, dict_step_to_state_cr, \
            dict_step_to_state_lon_ref, dict_step_to_state_lat_ref, \
            dict_step_to_state_lon_ego, dict_step_to_state_lat_ego, \
            dict_step_to_state_signal, dict_step_to_list_ids_lanelets, \
            incoming_element, direction_outgoing, set_ids_lanelets_outgoing_left, \
            set_ids_lanelets_outgoing_straight, set_ids_lanelets_outgoing_right, set_ids_lanelets_oncoming, \
            dict_step_to_sonia_prediction_occupancy

    @staticmethod
    def convert_to_curvilinear_state(state_cr: State, state_cr_previous: State, dt: float,
                                     route_ref: Route, CLCS_ref: CurvilinearCoordinateSystem,
                                     lane_ego: Lane, CLCS_ego: CurvilinearCoordinateSystem):
        """Converts the CommonRoad state into Curvilinear states under the reference and ego coordinate system."""
        # compute acceleration
        position = state_cr.position
        try:
            velocity = state_cr.velocity

        except AttributeError:
            velocity = 0

        try:
            velocity_previous = state_cr_previous.velocity

        except AttributeError:
            velocity_previous = 0

        orientation = state_cr.orientation
        acceleration = util_vehicle.compute_acceleration(velocity_previous, velocity, dt)

        # convert to state in the reference path frame
        try:
            s_ref, d_ref = CLCS_ref.convert_to_curvilinear_coords(position[0], position[1])

        except ValueError:
            state_lon_ref = state_lat_ref = None

        else:
            theta_at_s_ref = route_ref.orientation(s_ref)
            theta_ref = orientation - theta_at_s_ref
            velocity_lon_ref = velocity * np.cos(theta_ref)
            if acceleration:
                state_lon_ref = StateLongitudinal(s=s_ref, v=velocity_lon_ref, a=acceleration)

            else:
                state_lon_ref = StateLongitudinal(s=s_ref, v=velocity_lon_ref)

            state_lat_ref = StateLateral(d=d_ref, theta=theta_ref)

        # convert to state in the ego (local) frame
        try:
            s_ego, d_ego = CLCS_ego.convert_to_curvilinear_coords(position[0], position[1])

        except ValueError:
            state_lon_ego = state_lat_ego = None

        else:
            theta_at_s_ego = lane_ego.orientation(s_ego)
            theta_ego = orientation - theta_at_s_ego
            velocity_lon_ego = velocity * np.cos(theta_ego)
            if acceleration:
                state_lon_ego = StateLongitudinal(s=s_ego, v=velocity_lon_ego, a=acceleration)

            else:
                state_lon_ego = StateLongitudinal(s=s_ego, v=velocity_lon_ego)

            state_lat_ego = StateLateral(d=d_ego, theta=theta_ego)

        return state_lon_ref, state_lat_ref, state_lon_ego, state_lat_ego
