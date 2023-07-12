from collections import defaultdict
from typing import Optional, Set, Dict, List, Callable, Tuple

import numpy as np
from shapely.geometry.polygon import Polygon
from commonroad.scenario.intersection import IntersectionIncomingElement
from commonroad.scenario.lanelet import LaneletNetwork, LaneletType, Lanelet
from commonroad.scenario.traffic_sign import TrafficSignIDZamunda
from commonroad_dc.pycrccosy import CurvilinearCoordinateSystem
from commonroad_reach.data_structure.reach.reach_polygon import ReachPolygon

from commonroad_reach_semantic.data_structure.config.outgoing_direction import OutgoingDirection
from commonroad_reach_semantic.data_structure.config.semantic_configuration import SemanticConfiguration
from commonroad_reach_semantic.data_structure.environment_model.lanelet_model import LaneletModel
from commonroad_reach_semantic.data_structure.rule.proposition_holder import MultiStepPropositionHolder
from commonroad_reach_semantic.data_structure.environment_model.road_network import RoadNetwork
from commonroad_reach_semantic.data_structure.environment_model.vehicle import Vehicle
from commonroad_reach_semantic.data_structure.rule.proposition import Proposition as P
from commonroad_reach_semantic.data_structure.rule.proposition import PropositionGroup as PG
import commonroad_reach_semantic.utility.geometry as util_geometry


class Region:
    """
    Class representing a region of the lanelet network
    A regions contains:
       1. polygon in Cartesian coordinate system
       2. polygon in curvilinear coordinate system
       3. optional: a list of axis-aligned bounding boxes
    """
    config: SemanticConfiguration
    step_end: int
    lanelet_model: LaneletModel
    road_network: RoadNetwork
    lanelet_network: LaneletNetwork
    incoming_element_route: IntersectionIncomingElement
    discard_region_small: bool
    area_polygon_desired_min: float
    buffer_polygon: float
    length_aabb_max: float
    distance_vertices_polygon_max: float
    CLCS: CurvilinearCoordinateSystem
    dict_id_lanelet_to_lanelet: dict
    dict_id_lanelet_to_polygon_cart: dict

    @classmethod
    def initialize(cls, config: SemanticConfiguration, lanelet_model: LaneletModel):
        cls.config = config
        cls.step_end = config.planning.step_start + config.planning.steps_computation
        cls.lanelet_model = lanelet_model
        cls.road_network = lanelet_model.road_network
        cls.lanelet_network = lanelet_model.road_network.lanelet_network
        cls.incoming_element_route = config.semantic_model.incoming_element_route
        cls.discard_region_small = config.semantic_model.discard_region_small
        cls.area_polygon_desired_min = config.semantic_model.area_polygon_desired_min
        cls.buffer_polygon = config.semantic_model.buffer_polygon
        cls.distance_vertices_polygon_max = config.semantic_model.distance_vertices_polygon_max
        cls.CLCS = config.planning.CLCS
        # size_buffer = config.vehicle.ego.radius_inflation if config.semantic_model.consider_ego_shape else -0.01
        cls.dict_id_lanelet_to_lanelet = {lanelet.lanelet_id: lanelet for lanelet in cls.lanelet_network.lanelets}

        cls.dict_id_lanelet_to_polygon_cart = dict()
        # handles lon and lat similarly
        cls.attach_inflated_polygon_to_lanelets_simple()
        # differentiates between lon and lat
        # cls.attach_inflated_polygon_to_lanelets(lanelet_model.set_lanelets_route_related)

    @classmethod
    def attach_inflated_polygon_to_lanelets_simple(cls):
        """
        Attaches an inflated polygon to lanelet representing the region.

        Adds an attribute to the lanelet object.
        """
        # inflate the lanelet polygons by the inflation radius of the ego vehicle
        size_buffer = cls.config.vehicle.ego.radius_inflation if cls.config.semantic_model.consider_ego_shape else -0.01
        for lanelet in cls.lanelet_network.lanelets:
            lanelet.polygon_cart = lanelet.polygon.shapely_object.buffer(size_buffer, join_style=2)

    @classmethod
    def attach_inflated_polygon_to_lanelets(cls, set_lanelets_route_related: Set[Lanelet]):
        """
        Attaches an inflated polygon to lanelet representing the region.

        Adds an attribute to the lanelet object. For each lanelet, construct a curvilinear coordinate system using the
        concatenation of its centerline and those of its predecessor and successor lanelets. If the pred/suc lanelets
        are not within the route-related lanelets, choose any pred/suc lanelets; otherwise, choose the one that are
        the route-related lanelets. A polygon is inflated by half-length in the longitudinal direction and half-width
        in the lateral direction.
        """
        set_ids_lanelets_route_related = {lanelet.lanelet_id for lanelet in set_lanelets_route_related}
        for lanelet in set_lanelets_route_related:
            vertices_center = lanelet.center_vertices
            set_ids_lanelets_pred_in_route = set(lanelet.predecessor).intersection(set_ids_lanelets_route_related)
            set_ids_lanelets_succ_in_route = set(lanelet.successor).intersection(set_ids_lanelets_route_related)

            # === predecessor
            if set_ids_lanelets_pred_in_route:
                id_lanelet_pred = list(set_ids_lanelets_pred_in_route)[0]

            elif lanelet.predecessor:
                id_lanelet_pred = lanelet.predecessor[0]

            else:
                id_lanelet_pred = None

            vertices_center_pred = []
            if id_lanelet_pred:
                lanelet_pred = cls.lanelet_network.find_lanelet_by_id(id_lanelet_pred)
                vertices_center_pred = list(lanelet_pred.center_vertices)

            else:
                p_interpolated = util_geometry.interpolate_vertex(vertices_center[0], vertices_center[1], 5.0)
                # append twice, since one will be removed when concatenating
                vertices_center_pred.append(p_interpolated)
                vertices_center_pred.append(p_interpolated)

            # === successor
            if set_ids_lanelets_succ_in_route:
                id_lanelet_succ = list(set_ids_lanelets_succ_in_route)[0]

            elif lanelet.successor:
                id_lanelet_succ = lanelet.successor[0]

            else:
                id_lanelet_succ = None

            vertices_center_succ = []
            if id_lanelet_succ:
                lanelet_succ = cls.lanelet_network.find_lanelet_by_id(id_lanelet_succ)
                vertices_center_succ = list(lanelet_succ.center_vertices)

            else:
                p_interpolated = util_geometry.interpolate_vertex(vertices_center[-1], vertices_center[-2], 5.0)
                vertices_center_succ.append(p_interpolated)

            # construct CLCS
            vertices_center_concat = vertices_center_pred[:-1] + list(vertices_center)[:-1] + vertices_center_succ
            CLCS_lanelet = CurvilinearCoordinateSystem(np.array(vertices_center_concat), 25, 0.1)

            # construct shapely polygon
            length_half_ego = cls.config.vehicle.ego.length / 2
            width_half_ego = cls.config.vehicle.ego.width / 2
            tuple_attr_vertex_init = None
            tuple_attr_vertex_final = None
            list_vertices_extended_left = list()
            list_vertices_extended_right = list()
            for index, vertex_center in enumerate(vertices_center):
                vertices_left = lanelet.left_vertices[index]
                vertices_right = lanelet.right_vertices[index]

                try:
                    s_center, _ = CLCS_lanelet.convert_to_curvilinear_coords(vertex_center[0],
                                                                                    vertex_center[1])
                    _, d_left = CLCS_lanelet.convert_to_curvilinear_coords(vertices_left[0], vertices_left[1])
                    _, d_right = CLCS_lanelet.convert_to_curvilinear_coords(vertices_right[0], vertices_right[1])

                except ValueError:
                    continue

                # extend by half width
                d_left = (d_left + width_half_ego) if d_left > 0 else (d_left - width_half_ego)
                try:
                    p_left_cart = CLCS_lanelet.convert_to_cartesian_coords(s_center, d_left)

                except ValueError:
                    pass

                else:
                    list_vertices_extended_left.append([p_left_cart[0], p_left_cart[1]])

                # extend by half width
                d_right = (d_right + width_half_ego) if d_right > 0 else (d_right - width_half_ego)
                try:
                    p_right_cart = CLCS_lanelet.convert_to_cartesian_coords(s_center, d_right)

                except ValueError:
                    pass

                else:
                    list_vertices_extended_right.append([p_right_cart[0], p_right_cart[1]])

                if index == 0:
                    tuple_attr_vertex_init = (s_center, d_left, d_right)

                elif index == len(list(vertices_center)) - 1:
                    tuple_attr_vertex_final = (s_center, d_left, d_right)

            # append additional vertices modelling the half-length of the ego vehicle
            s_center_init = tuple_attr_vertex_init[0] - length_half_ego
            s_center_final = tuple_attr_vertex_final[0] + length_half_ego

            p_left_init_cart = CLCS_lanelet.convert_to_cartesian_coords(s_center_init, tuple_attr_vertex_init[1])
            list_vertices_extended_left.insert(0, [p_left_init_cart[0], p_left_init_cart[1]])

            p_left_final_cart = CLCS_lanelet.convert_to_cartesian_coords(s_center_final, tuple_attr_vertex_final[1])
            list_vertices_extended_left.append([p_left_final_cart[0], p_left_final_cart[1]])

            p_right_init_cart = CLCS_lanelet.convert_to_cartesian_coords(s_center_init, tuple_attr_vertex_init[2])
            list_vertices_extended_right.insert(0, [p_right_init_cart[0], p_right_init_cart[1]])

            p_right_final_cart = CLCS_lanelet.convert_to_cartesian_coords(s_center_final, tuple_attr_vertex_final[2])
            list_vertices_extended_right.append([p_right_final_cart[0], p_right_final_cart[1]])

            # construct shapley polygon
            list_vertices_extended_right.reverse()
            list_vertices_polygon_cart = list_vertices_extended_left + list_vertices_extended_right
            # shapley polygon requires identical initial and final vertices
            list_vertices_polygon_cart.append(list_vertices_polygon_cart[0])

            polygon_shapely = Polygon(list_vertices_polygon_cart)
            # polygon_shapely.buffer(-0.01, join_style=2)

            lanelet.polygon_cart = polygon_shapely

            cls.dict_id_lanelet_to_polygon_cart[lanelet.lanelet_id] = lanelet.polygon_cart

    def __init__(self, tuple_ids_lanelets=None, polygon: ReachPolygon = None, coordinate_system: str = "CART"):
        self.set_ids_lanelets = set(tuple_ids_lanelets)
        self.polygon_cart: Optional[ReachPolygon] = None
        self.polygon_cvln: Optional[ReachPolygon] = None
        self.list_aabbs = None

        if coordinate_system == "CART":
            self.polygon_cart = polygon.clone(convexify=False)

        elif coordinate_system == "CVLN":
            self.polygon_cvln = polygon.clone(convexify=False)

        else:
            raise Exception("Invalid coordinate system provided.")

        self.set_lanes = set()
        self.set_traffic_lights = set()
        self.set_traffic_lights_active = set()
        self.set_traffic_signs = set()

        self.dict_direction_to_priority = {}
        self.dict_id_lanelet_to_priorities = defaultdict(dict)
        self.set_ids_obstacles = set()
        self.set_vehicles_lanelet_outgoing = set()

        # for regions at intersections
        self.incoming_element = None
        self.set_ids_lanelets_outgoing_left = set()
        self.set_ids_lanelets_outgoing_straight = set()
        self.set_ids_lanelets_outgoing_right = set()
        self.oncoming_element = None
        self.set_ids_lanelets_oncoming = set()

        self.proposition_holder = MultiStepPropositionHolder(self.step_end + 1)
        self._complete_attributes()

    def __repr__(self):
        p_lon_min, p_lat_min, p_lon_max, p_lat_max = self.polygon_cvln.bounds
        return f"Region(IDs={self.set_ids_lanelets}, Lon:[{p_lon_min:.3f},{p_lon_max:.3f}], " \
               f"Lat:[{p_lat_min:.3f},{p_lat_max:.3f}], TI Propositions:{self.proposition_holder.time_invariant_propositions()})"

    def _complete_attributes(self):
        """
        Completes the attributes of a region.
        """
        self._update_lanes()
        self._update_traffic_lights_and_signs()
        self._update_relevant_obstacles()
        self._determine_intersection_attributes()
        self._label_time_invariant_propositions()

    @property
    def bounding_box_cart(self):
        return self.polygon_cart.bounds

    @property
    def bounding_box_cvln(self):
        return self.polygon_cvln.bounds

    def outgoing_lanelet_ids(self, direction: OutgoingDirection) -> Set[int]:
        match direction:
            case OutgoingDirection.LEFT:
                return self.set_ids_lanelets_outgoing_left
            case OutgoingDirection.STRAIGHT:
                return self.set_ids_lanelets_outgoing_straight
            case OutgoingDirection.RIGHT:
                return self.set_ids_lanelets_outgoing_right

    def assign_polygon(self, polygon: ReachPolygon, coordinate: str):
        """Assigns the polygon of the region"""
        if coordinate == "CART":
            self.polygon_cart = polygon.clone(convexify=False)

        elif coordinate == "CVLN":
            self.polygon_cvln = polygon.clone(convexify=False)

        else:
            raise Exception("<Region> Invalid coordinate system provided.")

    def intersects(self, coordinates_box, coordinate_system: str = "CVLN"):
        """
        Returns true if the input box intersects with the bounding box.
        """
        p_lon_min_box, p_lat_min_box, p_lon_max_box, p_lat_max_box = coordinates_box

        if coordinate_system == "CART":
            p_lon_min, p_lat_min, p_lon_max, p_lat_max = self.bounding_box_cart

        elif coordinate_system == "CVLN":
            p_lon_min, p_lat_min, p_lon_max, p_lat_max = self.bounding_box_cvln

        else:
            raise Exception("<Region> Invalid coordinate system provided.")

        if p_lon_max_box < p_lon_min or p_lon_min_box > p_lon_max \
                or p_lat_max_box < p_lat_min or p_lat_min_box > p_lat_max:
            return False

        else:
            return True

    def propositions_at_step(self, step: int) -> Set:
        return self.proposition_holder.propositions_at_step(step)

    def dict_group_to_propositions_at_step(self, step: int) -> Dict[PG, Set[str]]:
        return self.proposition_holder.dict_group_to_propositions_at_step(step)

    def colliding_with_vehicles(self):
        for proposition in self.proposition_holder.time_invariant_propositions():
            if "AlignedWith_" in proposition:
                id_vehicle = int(proposition.split("_")[1])

                if f"Beside_{id_vehicle}" in self.proposition_holder.time_invariant_propositions():
                    return True

        return False

    def _update_lanes(self):
        """
        Updates the set of lanes of the region.
        """
        for id_lanelet in self.set_ids_lanelets:
            self.set_lanes.update(self.road_network.find_lanes_by_lanelets({id_lanelet}))

    def _update_traffic_lights_and_signs(self):
        """
        Updates traffic lights and traffic signs of the region.
        """
        for id_lanelet in self.set_ids_lanelets:
            lanelet = self.dict_id_lanelet_to_lanelet[id_lanelet]

            for id_traffic_sign in lanelet.traffic_signs:
                traffic_sign = self.lanelet_network.find_traffic_sign_by_id(id_traffic_sign)
                self.set_traffic_signs.add(traffic_sign)

            for traffic_light in lanelet.traffic_lights:
                traffic_light = self.lanelet_network.find_traffic_light_by_id(traffic_light)
                self.set_traffic_lights.add(traffic_light)

        for traffic_light in self.set_traffic_lights:
            if traffic_light.active:
                self.set_traffic_lights_active.add(traffic_light)

    def _update_relevant_obstacles(self):
        """
        Updates relevant obstacles in the region.
        """
        for id_lanelet in self.set_ids_lanelets:
            lanelet = self.dict_id_lanelet_to_lanelet[id_lanelet]

            self.set_ids_obstacles.update(lanelet.static_obstacles_on_lanelet)
            for set_obstacles in lanelet.dynamic_obstacles_on_lanelet.values():
                self.set_ids_obstacles.update(set_obstacles)

    def _determine_intersection_attributes(self):
        """
        Determines the intersection-related attributes of the region.
        """
        list_incomings = []
        dict_incoming_to_intersection = dict()
        for intersection in self.lanelet_network.intersections:
            list_incomings += intersection.incomings
            for incoming in intersection.incomings:
                dict_incoming_to_intersection[incoming] = intersection

        # determine incoming element
        for incoming in list_incomings:
            if incoming.incoming_lanelets.intersection(self.set_ids_lanelets):
                self.incoming_element = incoming
                break

        if self.incoming_element:
            # determine outgoings
            for id_lanelet in self.incoming_element.successors_left:
                lanelet = self.dict_id_lanelet_to_lanelet[id_lanelet]
                if lanelet and lanelet.successor:
                    self.set_ids_lanelets_outgoing_left.update(set(lanelet.successor))

            for id_lanelet in self.incoming_element.successors_straight:
                lanelet = self.dict_id_lanelet_to_lanelet[id_lanelet]
                if lanelet and lanelet.successor:
                    self.set_ids_lanelets_outgoing_straight.update(set(lanelet.successor))

            for id_lanelet in self.incoming_element.successors_right:
                lanelet = self.dict_id_lanelet_to_lanelet[id_lanelet]
                if lanelet and lanelet.successor:
                    self.set_ids_lanelets_outgoing_right.update(set(lanelet.successor))

            # determine oncoming element and lanelets
            incoming_on_the_right = None
            intersection = dict_incoming_to_intersection[self.incoming_element]
            for incoming in intersection.incomings:
                if incoming.incoming_id == self.incoming_element.left_of:
                    incoming_on_the_right = incoming
                    break

            if incoming_on_the_right:
                for incoming in intersection.incomings:
                    if incoming.incoming_id == incoming_on_the_right.left_of:
                        self.oncoming_element = incoming
                        break

            if self.oncoming_element:
                self.set_ids_lanelets_oncoming.update(self.oncoming_element.incoming_lanelets)
                self.set_ids_lanelets_oncoming.update(self.oncoming_element.successors_straight)
                for id_lanelet in self.oncoming_element.successors_straight:
                    lanelet = self.dict_id_lanelet_to_lanelet[id_lanelet]
                    if lanelet and lanelet.successor:
                        self.set_ids_lanelets_oncoming.update(lanelet.successor)

    def _label_time_invariant_propositions(self):
        """
        Updates time invariant propositions of the region.
        """
        self._label_lanelet_propositions()
        self._label_traffic_light_propositions()

    def _label_lanelet_propositions(self):
        """
        Updates lanelet-related propositions.
        """
        for id_lanelet in self.set_ids_lanelets:
            self.proposition_holder.add_proposition(P.in_lanelet(id_lanelet), group=PG.POSITION)
            if id_lanelet in self.lanelet_model.main_carriageway_lanelet_ids:
                self.proposition_holder.add_proposition(P.on_main_carriageway(), group=PG.POSITION)
            if id_lanelet in self.lanelet_model.right_lane_lanelet_ids:
                self.proposition_holder.add_proposition(P.on_right_lane(), group=PG.POSITION)

    def _label_traffic_light_propositions(self):
        """
        Updates traffic-light-related propositions.
        """
        # 720 green arrow
        for traffic_sign in self.set_traffic_signs:
            for element in traffic_sign.traffic_sign_elements:
                if element.traffic_sign_element_id == TrafficSignIDZamunda.GREEN_ARROW:
                    self.proposition_holder.add_proposition(P.at_green_arrow(), group=PG.TRAFFIC_LIGHT)

        # examine whether there are relevant traffic lights in the upcoming lanelets
        set_id_lanelets_upcoming_intersection = set()
        # find all lanelets to be examined
        for lane in self.set_lanes:
            for id_lanelet in lane.list_ids_lanelets:
                lanelet = self.dict_id_lanelet_to_lanelet[id_lanelet]

                set_id_lanelets_upcoming_intersection.add(id_lanelet)
                # the examination stops before intersection. incoming is also considered an intersection (check this)
                if LaneletType.INTERSECTION in lanelet.lanelet_type:
                    break
        # check for relevant traffic lights
        has_relevant_traffic_light = False
        for id_lanelet in set_id_lanelets_upcoming_intersection:
            lanelet = self.dict_id_lanelet_to_lanelet[id_lanelet]

            set_ids_traffic_lights = lanelet.traffic_lights
            for id_traffic_light in set_ids_traffic_lights:
                traffic_light = self.lanelet_network.find_traffic_light_by_id(id_traffic_light)
                if traffic_light.active:
                    has_relevant_traffic_light = True
                    break

        if has_relevant_traffic_light:
            self.proposition_holder.add_proposition(P.has_relevant_traffic_light(), group=PG.TRAFFIC_LIGHT)

    def determine_priorities(self, dict_traffic_sign_to_priorities: Dict):
        """
        Determines traffic priorities based on the given dictionary.
        """
        priority_default = 3  # default priority from traffic sign 102 (right before left)
        # obtain priorities for lanelets in the region
        for id_lanelet in self.set_ids_lanelets:
            index_min = np.inf
            element_index_min = None
            lanelet = self.dict_id_lanelet_to_lanelet[id_lanelet]

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
                for direction in OutgoingDirection:
                    self.dict_id_lanelet_to_priorities[id_lanelet][direction] = \
                        dict_traffic_sign_to_priorities[element_index_min.traffic_sign_element_id][direction]

            except (KeyError, AttributeError):
                for direction in OutgoingDirection:
                    self.dict_id_lanelet_to_priorities[id_lanelet][direction] = priority_default

        # obtain priorities for the region
        index_min = np.inf
        element_index_min = None
        for traffic_sign in self.set_traffic_signs:
            for element in traffic_sign.traffic_sign_elements:
                try:
                    index = dict_traffic_sign_to_priorities[element.traffic_sign_element_id]["index"]

                except KeyError:
                    index = 20

                if index < index_min:
                    index_min = index
                    element_index_min = element

        try:
            for direction in OutgoingDirection:
                self.dict_direction_to_priority[direction] = \
                    dict_traffic_sign_to_priorities[element_index_min.traffic_sign_element_id][direction]

        except (KeyError, AttributeError):
            for direction in OutgoingDirection:
                self.dict_direction_to_priority[direction] = priority_default

    def examine_priorities_against_vehicles(self, list_vehicles: List[Vehicle]):
        """
        Examines priorities of the region against the given list of vehicles over time and vice versa.
        """
        # this is to store vehicles that don't have the same priority as the region
        dict_priority_evaluation = defaultdict(dict)
        self._examine_region_priority_over_vehicles(list_vehicles, dict_priority_evaluation)
        self._examine_vehicles_priority_over_region(list_vehicles, dict_priority_evaluation)
        self._add_same_priority_propositions(dict_priority_evaluation)

    def _examine_region_priority_over_vehicles(self, list_vehicles: List[Vehicle],
                                               dict_priority_evaluation: Dict[int, Dict[Tuple[OutgoingDirection, OutgoingDirection], List]]):
        """Examines the priorities of the region against the given list of vehicles.

        dict_priority_evaluation maps time step to tuple of directions to list of vehicle ids.
        """
        for vehicle in list_vehicles:
            for step in self.proposition_holder.time_variant_propositions():
                if not vehicle.lanelet_ids_at_step(step):
                    continue

                for direction_region in OutgoingDirection:
                    for direction_vehicle in OutgoingDirection:
                        tuple_directions = (direction_region, direction_vehicle)
                        if tuple_directions not in dict_priority_evaluation[step]:
                            dict_priority_evaluation[step][tuple_directions] = []

                        region_has_higher_priority = \
                            self._examine_region_priority_over_vehicle(vehicle, step, tuple_directions)
                        if not region_has_higher_priority:
                            dict_priority_evaluation[step][tuple_directions].append(vehicle.id_vehicle)

    def _examine_region_priority_over_vehicle(self, vehicle: Vehicle, step: int, tuple_directions: Tuple[OutgoingDirection, OutgoingDirection]) -> bool:
        """Returns true if the region has a higher priority in the specified direction over the vehicle."""
        direction_region = tuple_directions[0]
        direction_vehicle = tuple_directions[1]

        for id_lanelet_region, dict_priorities_lanelet in self.dict_id_lanelet_to_priorities.items():
            priority_lanelet_r = dict_priorities_lanelet[direction_region]

            region_has_higher_priority = True
            for id_lanelet_vehicle in vehicle.lanelet_ids_at_step(step):
                priority_lanelet_v = vehicle.dict_id_lanelet_to_priorities[id_lanelet_vehicle][direction_vehicle]
                region_has_higher_priority = region_has_higher_priority and (priority_lanelet_r > priority_lanelet_v)
            # if one of the region's lanelets has a higher priority over all lanelets of the vehicle
            if region_has_higher_priority:
                self.proposition_holder.add_proposition(
                    P.has_priority(vehicle.id_vehicle, direction_region, direction_vehicle),
                    PG.PRIORITY,
                    step
                )
                return True

        return False

    def _examine_vehicles_priority_over_region(self, list_vehicles: List[Vehicle],
                                               dict_priority_evaluation: Dict[int, Dict[Tuple[OutgoingDirection, OutgoingDirection], List]]):
        """Examines the priorities of the given list of vehicles against the region.

        dict_priority_evaluation maps time step to tuple of directions to list of vehicle ids.
        """
        for vehicle in list_vehicles:
            for step in self.proposition_holder.time_variant_propositions():
                if not vehicle.lanelet_ids_at_step(step):
                    continue

                for direction_region in OutgoingDirection:
                    for direction_vehicle in OutgoingDirection:
                        tuple_directions = (direction_region, direction_vehicle)

                        vehicle_has_higher_priority = \
                            self.examine_vehicle_priority_over_region(vehicle, step, tuple_directions)
                        if vehicle_has_higher_priority:
                            dict_priority_evaluation[step][tuple_directions].remove(vehicle.id_vehicle)

    def examine_vehicle_priority_over_region(self, vehicle: Vehicle, step: int, tuple_directions: Tuple[OutgoingDirection, OutgoingDirection]):
        """Returns true if the vehicle has a higher priority in the specified direction over the region."""
        direction_region = tuple_directions[0]
        direction_vehicle = tuple_directions[1]

        for id_lanelet_vehicle in vehicle.lanelet_ids_at_step(step):
            priority_lanelet_v = vehicle.dict_id_lanelet_to_priorities[id_lanelet_vehicle][direction_vehicle]

            vehicle_has_higher_priority = True
            for id_lanelet_region, dict_priorities_lanelet in self.dict_id_lanelet_to_priorities.items():
                priority_lanelet_r = dict_priorities_lanelet[direction_region]
                vehicle_has_higher_priority = vehicle_has_higher_priority and (priority_lanelet_v > priority_lanelet_r)
            # if one of the vehicle's lanelets has a higher priority over all lanelets of the region
            if vehicle_has_higher_priority:
                self.proposition_holder.add_proposition(
                    P.no_priority(vehicle.id_vehicle, direction_region, direction_vehicle),
                    PG.PRIORITY,
                    step
                )
                return True

        return False

    def _add_same_priority_propositions(self, dict_priority_evaluation: Dict[int, Dict[Tuple[OutgoingDirection, OutgoingDirection], List]]):
        """Adds propositions indicating that the region has the same priority as the vehicles.

         dict_priority_evaluation maps time step to tuple of directions to list of vehicle ids.
         """
        for step, dict_tuple_directions_to_list_ids_vehicles in dict_priority_evaluation.items():
            for tuple_directions, list_ids_vehicles in dict_tuple_directions_to_list_ids_vehicles.items():
                direction_region = tuple_directions[0]
                direction_vehicle = tuple_directions[1]

                for id_vehicle in list_ids_vehicles:
                    self.proposition_holder.add_proposition(
                        P.same_priority(id_vehicle, direction_region, direction_vehicle),
                        PG.PRIORITY,
                        step
                    )

    def construct_aabbs(self):
        """Creates a list of axis-aligned bounding boxes for the curvilinear polygon

        The polygon is first split in the longitudinal direction, then the lateral.
        """
        if self.colliding_with_vehicles():
            return None

        area_aabb_acceptable = self.length_aabb_max * self.length_aabb_max * 0.2
        p_lon_min, p_lat_min, p_lon_max, p_lat_max = self.bounding_box_cvln

        # return if both the lon and lat length of the polygon is less than the max length
        if (p_lon_max - p_lon_min) < self.length_aabb_max and (p_lat_max - p_lat_min) < self.length_aabb_max:
            return [ReachPolygon.from_rectangle_vertices(*self.bounding_box_cvln)]

        # at least one direction can be split
        list_polygons = [self.polygon_cvln]
        list_polygons_temp = []
        # consider longitudinal length
        for polygon in list_polygons:
            p_lon_min, p_lat_min, p_lon_max, p_lat_max = polygon.bounds
            length_lon = p_lon_max - p_lon_min

            if length_lon > self.length_aabb_max:
                # create partitions for the polygon
                num_partitions = int(length_lon // self.length_aabb_max)
                # 2.1m / 1.0m = 3 partitions, 2.0m / 1.0m = 2 partitions
                num_partitions = num_partitions + 1 if length_lon % self.length_aabb_max else num_partitions
                for idx_partition in range(num_partitions):
                    p_lon_min_new = p_lon_min + idx_partition * self.length_aabb_max
                    p_lon_max_new = min(p_lon_max, p_lon_min + (idx_partition + 1) * self.length_aabb_max)

                    polygon_partition = ReachPolygon.from_rectangle_vertices(
                        p_lon_min_new, p_lat_min, p_lon_max_new, p_lat_max).shapely_object.buffer(+0.01)
                    polygon_partition = polygon_partition.intersection(polygon.shapely_object.buffer(+0.01))

                    if not polygon_partition.is_empty:
                        list_polygons_temp.append(polygon_partition)

            else:
                # the longitudinal length does not exceed the max length, thus no partitioning required
                list_polygons_temp.append(polygon)

        list_polygons = list_polygons_temp
        list_polygons_final = []
        # consider lateral length
        for polygon in list_polygons:
            p_lon_min, p_lat_min, p_lon_max, p_lat_max = polygon.bounds
            length_lat = p_lat_max - p_lat_min

            if length_lat > self.length_aabb_max:
                # create partitions for the polygon
                num_partitions = int(length_lat // self.length_aabb_max)
                num_partitions = num_partitions + 1 if length_lat % self.length_aabb_max else num_partitions
                for idx_partition in range(num_partitions):
                    p_lat_min_new = p_lat_min + idx_partition * self.length_aabb_max
                    p_lat_max_new = min(p_lat_max, p_lat_min + (idx_partition + 1) * self.length_aabb_max)

                    polygon_partition = ReachPolygon.from_rectangle_vertices(
                        p_lon_min, p_lat_min_new, p_lon_max, p_lat_max_new).shapely_object.buffer(+0.01)
                    polygon_partition = polygon_partition.intersection(polygon.shapely_object.buffer(+0.01))

                    if not polygon_partition.is_empty:
                        list_polygons_final.append(polygon_partition)

            else:
                # the lateral length does not exceed the max length, thus no partitioning required
                list_polygons_final.append(polygon)

        # turn polygons into aabbs by over-approximation
        list_aabbs = [ReachPolygon.from_rectangle_vertices(*polygon.bounds) for polygon in list_polygons_final
                      if polygon.area > area_aabb_acceptable]

        self.list_aabbs = list_aabbs
