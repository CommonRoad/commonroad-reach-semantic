import logging
import warnings
from collections import defaultdict
from typing import List, Dict, Union

import commonroad_reach.utility.logger as util_logger
import numpy as np
from commonroad.scenario.lanelet import LaneletNetwork, LaneletType
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.scenario.traffic_sign import TrafficLightDirection, TrafficLightState

import commonroad_reach_semantic.utility.reach_operation as reach_operation
import commonroad_reach_semantic.utility.region as util_region
from commonroad_reach_semantic.data_structure.position_interval import PositionInterval
from commonroad_reach_semantic.data_structure.proposition import Proposition as P
from commonroad_reach_semantic.data_structure.proposition import PropositionGroup as PG
from commonroad_reach_semantic.data_structure.reach.semantic_reach_node import SemanticReachNode
from commonroad_reach_semantic.data_structure.region import Region
from commonroad_reach_semantic.data_structure.road_network import RoadNetwork
from commonroad_reach_semantic import pycrreachs
from commonroad_reach_semantic.data_structure.semantic_configuration import SemanticConfiguration
# from commonroad_reach_semantic.data_structure.sonia_interface import SONIAInterface
from commonroad_reach_semantic.data_structure.vehicle import Vehicle

logger = logging.getLogger(__name__)


class SemanticModel:
    """
    Class to represent the semantic model of a given CommonRoad scenario.
    """

    def __init__(self, config: SemanticConfiguration):
        """
        Steps:
            1. create a smaller lanelet network and build a road network from it
            2. create vehicle objects from dynamic obstacles within the fov of the ego vehicle
            3. extract longitudinal/lateral position intervals from vehicles
            4. create lanelet regions
            5. extract traffic status propositions
        """
        logger.info("Creating SemanticModel...")

        self.config = config
        self.step_start = self.config.planning.step_start
        self.step_end = self.step_start + self.config.planning.steps_computation

        # lanelet-related
        self.set_lanelets_route_related = set()
        self.set_ids_lanelets_same_direction = set()
        self.set_ids_lanelets_opposite_direction = set()
        self.set_ids_lanelets_in_intersections = set()
        self.local_lanelet_network = None
        self.road_network = None
        self.scenario_with_sonia = None
        self.dict_id_lanelet_to_lanelet = dict()
        self.dict_id_lanelet_to_set_ids_lanelets_intersecting = defaultdict(set)

        # vehicle-related
        self.list_vehicles: List[Vehicle] = list()
        self.set_ids_vehicles_entering_intersection = set()
        self.dict_sonia_prediction = defaultdict(dict)

        # region-related
        self.list_regions: List[Region] = list()
        self.dict_step_to_position_intervals = dict()

        # proposition-related
        self.dict_step_to_traffic_status_propositions = dict()

        self._create_local_lanelet_network_and_road_network()
        self._create_vehicles()
        self._create_position_intervals()
        self._create_lanelet_regions()
        self._determine_propositions()

        logger.info("SemanticModel created.")
        self.print_summary()

    def print_summary(self):
        string = "# ========= Model Summary ========= #\n"
        string += f"#\tLanes: {len(self.road_network.list_lanes)}\n"
        string += f"#\tVehicles: {len(self.list_vehicles)}\n"
        string += f"#\tRegions: {len(self.list_regions)}\n"
        string += "# ================================= #"

        for line in string.split("\n"):
            util_logger.print_and_log_info(logger, line)

    def _create_local_lanelet_network_and_road_network(self):
        """
        Constructs a local lanelet network from lanelets close to the route lanelets.

        This is to speed up later computations.
        """
        # obtain lanelets related to the planned route, and sets of lanelet ids in its same and opposite directions
        self.set_lanelets_route_related, self.set_ids_lanelets_same_direction, self.set_ids_lanelets_opposite_direction = \
            self._obtain_route_related_lanelets()

        # obtain lanelets in the proximity of the planned route
        set_lanelets_in_proximity = self._obtain_lanelets_in_proximity_of_route()

        # create a local lanelet network to be considered in the semantic model
        self.local_lanelet_network = \
            self._create_local_lanelet_network(set_lanelets_in_proximity.union(self.set_lanelets_route_related))

        # retrieve ids of lanelets in intersections
        self.set_ids_lanelets_in_intersections = self._extract_lanelets_in_intersections()

        # create a road network to compute lanes in the scenario
        self.road_network = RoadNetwork(self.local_lanelet_network)

        # create dictionary mapping id to lanelet
        for lanelet in self.local_lanelet_network.lanelets:
            self.dict_id_lanelet_to_lanelet[lanelet.lanelet_id] = lanelet

        # cache intersection
        for lanelet_1 in self.local_lanelet_network.lanelets:
            for lanelet_2 in self.local_lanelet_network.lanelets:
                if reach_operation.are_intersecting_lanelets(lanelet_1, lanelet_2):
                    self.dict_id_lanelet_to_set_ids_lanelets_intersecting[lanelet_1.lanelet_id].add(
                        lanelet_2.lanelet_id)

        logger.info("Sub-lanelet network and road network created.")

    def _obtain_route_related_lanelets(self):
        """
        Returns a list of relevant lanelets in the scenario ot be considered.

        We first obtain the list of lanelets of the route, then iteratively add their adjacent lanelets.
        """
        set_ids_lanelets = set(self.config.planning.route.list_ids_lanelets)

        # obtain lanelets in the same direction as the route
        terminate = False
        while not terminate:
            num_ids_lanelets = len(set_ids_lanelets)
            for id_lanelet in list(set_ids_lanelets):
                lanelet = self.config.scenario.lanelet_network.find_lanelet_by_id(id_lanelet)

                # if left lanelet is in the same direction
                if lanelet.adj_left and lanelet.adj_left_same_direction:
                    set_ids_lanelets.add(lanelet.adj_left)

                # if right lanelet is in the same direction
                if lanelet.adj_right and lanelet.adj_right_same_direction:
                    set_ids_lanelets.add(lanelet.adj_right)

            terminate = (num_ids_lanelets == len(set_ids_lanelets))

        set_ids_lanelets_same_direction = set_ids_lanelets.copy()

        # obtain lanelets in both same and opposite directions
        terminate = False
        while not terminate:
            num_ids_lanelets = len(set_ids_lanelets)
            for id_lanelet in list(set_ids_lanelets):
                lanelet = self.config.scenario.lanelet_network.find_lanelet_by_id(id_lanelet)

                if lanelet.adj_left:
                    set_ids_lanelets.add(lanelet.adj_left)

                if lanelet.adj_right:
                    set_ids_lanelets.add(lanelet.adj_right)

            terminate = (num_ids_lanelets == len(set_ids_lanelets))

        set_ids_lanelets_opposite_direction = set_ids_lanelets.difference(set_ids_lanelets_same_direction)

        set_lanelets_route_related = {self.config.scenario.lanelet_network.find_lanelet_by_id(id_lanelet)
                                      for id_lanelet in set_ids_lanelets}

        # add predecessors/successors if required
        set_lanelets_to_add = set()
        if self.config.semantic_model.consider_route_predecessor:
            for lanelet in set_lanelets_route_related:
                for id_lanelet_predecessor in lanelet.predecessor:
                    lanelet_predecessor = \
                        self.config.scenario.lanelet_network.find_lanelet_by_id(id_lanelet_predecessor)
                    set_lanelets_to_add.add(lanelet_predecessor)

        if self.config.semantic_model.consider_route_successor:
            for lanelet in set_lanelets_route_related:
                for id_lanelet_successor in lanelet.successor:
                    lanelet_successor = self.config.scenario.lanelet_network.find_lanelet_by_id(id_lanelet_successor)
                    set_lanelets_to_add.add(lanelet_successor)

        set_lanelets_route_related.update(set_lanelets_to_add)

        return set_lanelets_route_related, set_ids_lanelets_same_direction, set_ids_lanelets_opposite_direction

    def _obtain_lanelets_in_proximity_of_route(self):
        list_lanelets_route = [self.config.scenario.lanelet_network.find_lanelet_by_id(id_lanelet)
                               for id_lanelet in self.config.planning.route.list_ids_lanelets]
        # get the coordinates of the bounding box
        list_vertices = []
        for lanelet in list_lanelets_route:
            for vertex in lanelet.center_vertices:
                list_vertices.append(vertex)

        x_min = min([x for x, y in list_vertices])
        x_max = max([x for x, y in list_vertices])
        y_min = min([y for x, y in list_vertices])
        y_max = max([y for x, y in list_vertices])
        vertex_circle = np.array([(x_max + x_min) / 2.0, (y_max + y_min) / 2.0])
        radius_circle = max(x_max - x_min, y_max - y_min)

        return set(self.config.scenario.lanelet_network.lanelets_in_proximity(vertex_circle, radius_circle))

    def _create_local_lanelet_network(self, set_lanelets):
        """
        Returns a lanelet network with the given set of lanelets.
        """
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            local_lanelet_network = LaneletNetwork.create_from_lanelet_network(self.config.scenario.lanelet_network)

            # clear existing lanelets
            for lanelet in local_lanelet_network.lanelets:
                local_lanelet_network.remove_lanelet(lanelet.lanelet_id)

            # add lanelets
            for lanelet in set_lanelets:
                if lanelet not in local_lanelet_network.lanelets:
                    local_lanelet_network.add_lanelet(lanelet)

            local_lanelet_network.cleanup_lanelet_references()

        return local_lanelet_network

    def _extract_lanelets_in_intersections(self):
        """
        Returns a set of intersection lanelet ids.
        """
        set_ids_lanelets_intersection = set()
        for intersection in self.config.scenario.lanelet_network.intersections:
            for incoming in intersection.incomings:
                set_ids_lanelets_intersection.update(incoming.successors_left)
                set_ids_lanelets_intersection.update(incoming.successors_straight)
                set_ids_lanelets_intersection.update(incoming.successors_right)

        return set_ids_lanelets_intersection

    def _create_vehicles(self):
        """
        Creates vehicle objects from relevant obstacles in the scenario.
        """
        Vehicle.initialize(self.config, self.road_network)

        if self.config.semantic_model.use_sonia:
            # self.scenario_with_sonia, self.dict_sonia_prediction = self._obtain_sonia_prediction()
            logger.error("SONIA not connected yet")

        list_obstacles_relevant = self._retrieve_relevant_obstacles(fov=self.config.vehicle.ego.fov)
        self._add_obstacles_to_lanelets(list_obstacles_relevant)
        self.list_vehicles = self._create_vehicles_from_obstacles(list_obstacles_relevant)
        self.set_ids_vehicles_entering_intersection = self._retrieve_vehicles_entering_intersection()

        logger.info("Vehicles created.")

    # def _obtain_sonia_prediction(self):
    #     """
    #     Returns a new scenario with automata prediction.
    #     """
    #     util_logger.print_and_log_info(logger, "* Computing SONIA Prediction...")
    #     sonia_interface = SONIAInterface(self.config)
    #     sonia_interface.predict_occupancies()
    #     dict_sonia_prediction = sonia_interface.postprocess_prediction()
    #     sonia_interface.deregister_scenario()
    #
    #     return sonia_interface.scenario, dict_sonia_prediction

    def _retrieve_relevant_obstacles(self, fov=200, bound_with_circle=True):
        """
        Returns a list of obstacles in the scenario to be considered in the computation.

        Computes a circle with the initial position as the center, and the fov of the ego vehicle as the radius.
        The vehicles within this radius are deemed as relevant obstacles.
        """
        list_obstacles_relevant = []

        if not bound_with_circle:
            # return all obstacles in the scenario
            return self.config.scenario.obstacles

        # obtain vehicles within the fov of the ego vehicle
        for obs in self.config.scenario.obstacles:
            # compute the distance between the initial position of ego and other vehicles
            dis = np.linalg.norm(obs.initial_state.position - self.config.planning_problem.initial_state.position)
            if dis <= fov:
                list_obstacles_relevant.append(obs)

        return list_obstacles_relevant

    def _add_obstacles_to_lanelets(self, list_obstacles):
        """
        Adds obstacles to lanelets.

        An obstacle is added to a lanelet if its occupancy in the future time steps intersects with the lanelet.
        """
        for obstacle in list_obstacles:
            for lanelet in self.local_lanelet_network.lanelets:
                polygon_lanelet = lanelet.polygon.shapely_object

                for step in range(self.step_end + 1):
                    time_step = step * round(self.config.planning.dt * 10)
                    occupancy = obstacle.occupancy_at_time(time_step)
                    if occupancy and occupancy.shape.shapely_object.intersects(polygon_lanelet):
                        if isinstance(obstacle, DynamicObstacle):
                            lanelet.add_dynamic_obstacle_to_lanelet(obstacle.obstacle_id, time_step)

                        else:
                            lanelet.add_static_obstacle_to_lanelet(obstacle.obstacle_id)
                            break

    def _create_vehicles_from_obstacles(self, list_obstacles):
        """
        Creates a list of vehicle objects from the given list of obstacles.
        """
        list_vehicles = []

        for obstacle in list_obstacles:
            vehicle = Vehicle.create_vehicle_from_obstacle(obstacle, self.dict_sonia_prediction)
            if vehicle:
                list_vehicles.append(vehicle)

        return list_vehicles

    def _retrieve_vehicles_entering_intersection(self):
        """
        Returns set of ids of vehicles entering intersection
        """
        set_ids_vehicle = set()

        for vehicle in self.list_vehicles:
            for id_lanelet in vehicle.lane.list_ids_lanelets:
                lanelet_vehicle = self.local_lanelet_network.find_lanelet_by_id(id_lanelet)

                if LaneletType.INTERSECTION in lanelet_vehicle.lanelet_type:
                    set_ids_vehicle.add(vehicle.id_vehicle)

        # alternatively, one can also check the lanelets a vehicle occupies during the planning horizon
        return set_ids_vehicle

    def _create_position_intervals(self):
        """
        Creates position intervals from vehicles.
        """
        # physical dimensions of the ego vehicle
        length_ego = self.config.vehicle.ego.length
        width_ego = self.config.vehicle.ego.width

        # position interval to be split w.r.t vehicles
        interval_lon_initial = PositionInterval(0 + length_ego / 2,
                                                self.config.planning.route.path_length[-1] - length_ego / 2, set())
        interval_lat_initial = PositionInterval(-self.config.semantic_model.p_lateral_max + width_ego / 2,
                                                self.config.semantic_model.p_lateral_max - width_ego / 2, set())
        # iterate through steps
        for step in range(self.step_end + 1):
            self.dict_step_to_position_intervals[step] = {"lon": [], "lat": []}
            list_intervals_lon = [interval_lon_initial.clone()]
            list_intervals_lat = [interval_lat_initial.clone()]

            for vehicle in self.list_vehicles:
                list_intervals_lon_split = []
                list_intervals_lat_split = []

                for interval_lon in list_intervals_lon:
                    list_intervals_lon_split += \
                        interval_lon.split_with_respect_to_vehicle(step, vehicle, length_ego / 2, "lon")

                for interval_lat in list_intervals_lat:
                    list_intervals_lat_split += \
                        interval_lat.split_with_respect_to_vehicle(step, vehicle, width_ego / 2, "lat")

                list_intervals_lon = list_intervals_lon_split
                list_intervals_lat = list_intervals_lat_split

            # sort longitudinal and lateral position intervals
            list_intervals_lon.sort(key=lambda interval: interval.p_min)
            list_intervals_lat.sort(key=lambda interval: interval.p_min)
            self.dict_step_to_position_intervals[step]["lon"] = list_intervals_lon
            self.dict_step_to_position_intervals[step]["lat"] = list_intervals_lat

        logger.info("Position intervals created.")

    def _create_lanelet_regions(self):
        """
        Constructs lanelet regions both in the Cartesian and curvilinear coordinate systems

        A lanelet region is a polygon which encloses all positions within the lanelet. The reachable sets are later
        cut down to lanelet regions for determining their position propositions.
        """
        Region.initialize(self.config, self.road_network, self.set_lanelets_route_related)
        set_tuples_ids_lanelets_intersecting = \
            util_region.detect_intersecting_lanelets(self.set_lanelets_route_related)

        list_regions = util_region.construct_regions_for_intersecting_lanelets(set_tuples_ids_lanelets_intersecting)

        # non-intersecting regions only appear if the shape of the ego vehicle is not considered
        if not self.config.semantic_model.consider_ego_shape:
            list_regions += \
                util_region.construct_regions_for_nonintersecting_lanelets(set_tuples_ids_lanelets_intersecting)

        self.list_regions = list_regions

        logger.info("Lanelet regions created.")

    def _determine_propositions(self):
        """
        Determines relevant propositions.
        """
        self._label_region_with_time_invariant_propositions()
        self._label_region_with_time_variant_propositions()
        self._determine_traffic_status_propositions()

        logger.info("Propositions determined.")

    def _label_region_with_time_invariant_propositions(self):
        """
        Labels regions with time invariant propositions.
        """
        self._label_driving_direction_propositions()
        self._label_lanelet_type_propositions()
        self._label_region_vehicle_intersection_incoming_propositions()
        self._label_region_vehicle_same_lane_propositions()

    def _label_driving_direction_propositions(self):
        """
        Labels regions with propositions related to driving directions.
        """
        for region in self.list_regions:
            if not region.set_ids_lanelets.intersection(self.set_ids_lanelets_opposite_direction):
                region.proposition_holder.add_proposition(P.same_driving_direction(), PG.TRAFFIC_SIGN)

    def _label_lanelet_type_propositions(self):
        """
        Labels regions with propositions related to lanelet types.
        """
        # in intersection
        for region in self.list_regions:
            if region.set_ids_lanelets.intersection(self.set_ids_lanelets_in_intersections):
                region.proposition_holder.add_proposition(P.in_intersection(), PG.POSITION)

        # in successor lanelets of the incoming element
        direction_outgoing = self.config.semantic_model.direction_outgoing
        if not direction_outgoing:
            return None

        for region in self.list_regions:
            # TODO: get rid of eval
            if region.set_ids_lanelets.intersection(
                    eval(f"self.config.semantic_model.incoming_element_route.successors_{direction_outgoing}")):
                region.proposition_holder.add_proposition(eval(f"P.in_{direction_outgoing}_successor()"), PG.POSITION)

    def _label_region_vehicle_intersection_incoming_propositions(self):
        """
        Updates the intersection incoming relations between the region and vehicles over time.

        This definition is different from the one in Sebastian's IV2022 paper. The proposition is propagated to
        successor lanelets of the incoming lanelets so that it is still present even after entering the intersection.
        """
        incoming_element_route = Region.incoming_element_route
        if not incoming_element_route:
            return None

        for vehicle in self.list_vehicles:
            incoming_element_vehicle = vehicle.incoming_element
            if not incoming_element_vehicle:
                continue

            # if the route's incoming element is left of vehicle's incoming element, propagate this to the incoming and
            # corresponding successor lanelets of the incoming element.
            if incoming_element_route.left_of == incoming_element_vehicle.incoming_id:
                set_ids_lanelets_effective = incoming_element_route.incoming_lanelets.union(
                    eval(f"incoming_element_route.successors_{self.config.semantic_model.direction_outgoing}"))

                for region in self.list_regions:
                    if region.set_ids_lanelets.intersection(set_ids_lanelets_effective):
                        region.proposition_holder.add_proposition(P.intersection_left_of(vehicle.id_vehicle),
                                                                  PG.INTERSECTION)

    def _label_region_vehicle_same_lane_propositions(self):
        """
        Updates the lane relation between the regions and the vehicles.
        """
        for region in self.list_regions:
            for vehicle in self.list_vehicles:
                lane_vehicle = vehicle.lane
                if lane_vehicle in region.set_lanes:
                    region.proposition_holder.add_proposition(P.in_same_lane(vehicle.id_vehicle), PG.VEHICLE)

    def _label_region_with_time_variant_propositions(self):
        """
        Updates time variant propositions of the region.
        """
        self._label_region_vehicle_intersection_oncoming_propositions()
        self._label_region_vehicle_outgoing_propositions()
        self._label_traffic_light_status_propositions()

    def _label_region_vehicle_intersection_oncoming_propositions(self):
        """
        Updates the intersection oncoming relations between the region and vehicles over time.
        """
        # examine if vehicle is on oncoming of the region
        for region in self.list_regions:
            incoming_region = region.incoming_element
            if not incoming_region:
                continue

            for vehicle in self.list_vehicles:
                for step in range(self.step_end + 1):
                    list_ids_lanelets_vehicle_at_step = vehicle.lanelet_ids_at_step(step)

                    if region.set_ids_lanelets_oncoming.intersection(list_ids_lanelets_vehicle_at_step):
                        region.proposition_holder.add_proposition(P.on_oncoming(vehicle.id_vehicle), PG.INTERSECTION,
                                                                  step)

        # examine if the region is on oncoming of the vehicle
        for region in self.list_regions:
            for vehicle in self.list_vehicles:
                if region.set_ids_lanelets.intersection(vehicle.set_ids_lanelets_oncoming):
                    region.proposition_holder.add_proposition(P.on_oncoming_of(vehicle.id_vehicle), PG.INTERSECTION)

    def _label_region_vehicle_outgoing_propositions(self):
        """
        Updates the intersection outgoing relations between the region the vehicles over time.
        """
        list_directions = ["left", "straight", "right"]
        for region in self.list_regions:
            for vehicle in self.list_vehicles:
                for step in range(self.step_end + 1):
                    for dir_region in list_directions:
                        for dir_vehicle in list_directions:
                            # TODO: get rid of eval
                            if eval(f"region.set_ids_lanelets_outgoing_{dir_region}").intersection(
                                    eval(f"vehicle.{dir_vehicle}_outgoings_at_step(step)")):
                                proposition = eval(f"P.{dir_region}_out_same_as_{dir_vehicle}_out(vehicle.id_vehicle)")
                                region.proposition_holder.add_proposition(proposition, PG.PRIORITY, step)

    def _label_traffic_light_status_propositions(self):
        """
        Updates traffic light status of the region.
        """
        for region in self.list_regions:
            for traffic_light in region.set_traffic_lights_active:
                for step in range(self.step_end + 1):
                    state_light = traffic_light.get_state_at_time_step(step)

                    # red left
                    if state_light in [TrafficLightState.RED, TrafficLightState.RED_YELLOW]:
                        if traffic_light.direction in [TrafficLightDirection.LEFT,
                                                       TrafficLightDirection.LEFT_STRAIGHT,
                                                       TrafficLightDirection.LEFT_RIGHT,
                                                       TrafficLightDirection.ALL]:
                            region.proposition_holder.add_proposition(P.at_red_left_traffic_light(), PG.TRAFFIC_LIGHT,
                                                                      step)

                    # red straight
                    if state_light in [TrafficLightState.RED, TrafficLightState.RED_YELLOW]:
                        if traffic_light.direction in [TrafficLightDirection.STRAIGHT,
                                                       TrafficLightDirection.LEFT_STRAIGHT,
                                                       TrafficLightDirection.STRAIGHT_RIGHT,
                                                       TrafficLightDirection.ALL]:
                            region.proposition_holder.add_proposition(P.at_red_straight_traffic_light(),
                                                                      PG.TRAFFIC_LIGHT,
                                                                      step)

                    # red right
                    if state_light in [TrafficLightState.RED, TrafficLightState.RED_YELLOW]:
                        if traffic_light.direction in [TrafficLightDirection.RIGHT,
                                                       TrafficLightDirection.STRAIGHT_RIGHT,
                                                       TrafficLightDirection.LEFT_RIGHT,
                                                       TrafficLightDirection.ALL]:
                            region.proposition_holder.add_proposition(P.at_red_right_traffic_light(),
                                                                      PG.TRAFFIC_LIGHT, step)

    def _determine_traffic_status_propositions(self):
        """
        Determines propositions for the general traffic status.
        """
        dict_step_to_traffic_status_propositions = defaultdict(set)

        # extract propositions indicating a vehicle is within an intersection
        for id_lanelet in self.set_ids_lanelets_in_intersections:
            lanelet = self.config.scenario.lanelet_network.find_lanelet_by_id(id_lanelet)

            for step in range(self.step_end + 1):
                set_ids_obstacles_dynamic = lanelet.dynamic_obstacle_by_time_step(step)

                if set_ids_obstacles_dynamic:
                    for id_obstacle in set_ids_obstacles_dynamic:
                        dict_step_to_traffic_status_propositions[step].add(P.in_intersection(id_obstacle))

        # extract propositions indicating a vehicle is in its outgoing lanelet
        for vehicle in self.list_vehicles:
            for step in range(self.step_end + 1):
                if vehicle.set_ids_lanelets_successor_incoming.intersection(vehicle.lanelet_ids_at_step(step)):
                    dict_step_to_traffic_status_propositions[step].add(
                        eval(f"P.in_{vehicle.type_outgoing}_successor(vehicle.id_vehicle)"))

        self.dict_step_to_traffic_status_propositions = dict_step_to_traffic_status_propositions

    def determine_traffic_priorities(self, dict_traffic_sign_to_priorities: Dict):
        """
        Determines the traffic priorities for regions and vehicles.
        """
        if self.config.semantic_model.incoming_element_route:
            # vehicles
            for vehicle in self.list_vehicles:
                vehicle.determine_priorities(dict_traffic_sign_to_priorities)

            # lanelet regions
            for region in self.list_regions:
                region.determine_priorities(dict_traffic_sign_to_priorities)
                region.examine_priorities_against_vehicles(self.list_vehicles)

            logger.info("Traffic priorities determined.")

    def find_vehicle_by_id(self, id_vehicle: int):
        """
        Returns the vehicle object by its id.
        """
        for vehicle in self.list_vehicles:
            if vehicle.id_vehicle == id_vehicle:
                return vehicle

    def label_traffic_propositions(self, step,
                                   list_propagated_sets: Union[List[SemanticReachNode], List[pycrreachs.SemanticReachNode]]) \
            -> Union[List[SemanticReachNode], List[pycrreachs.SemanticReachNode]]:
        """
        Labels propagated sets with propositions related to traffic status.
        """
        if not list_propagated_sets:
            return []

        list_propagated_sets = self.label_traffic_status_propositions(step, list_propagated_sets)
        list_propagated_sets = self.label_in_conflict_area_propositions(step, list_propagated_sets)
        list_propagated_sets = self.label_causes_braking_propositions(step, list_propagated_sets)

        return list_propagated_sets

    def label_traffic_status_propositions(self, step,
                                          list_propagated_sets: Union[List[SemanticReachNode], List[pycrreachs.SemanticReachNode]]) \
            -> Union[List[SemanticReachNode], List[pycrreachs.SemanticReachNode]]:
        """
        Labels propagated sets with traffic status propositions.
        """
        set_propositions = self.dict_step_to_traffic_status_propositions[step]
        # if isinstance(list_propagated_sets[0], ReachNode):
        for propagated_set in list_propagated_sets:
            propagated_set.proposition_holder.add_propositions(set_propositions, PG.TRAFFIC_STATUS)
        #
        # else:
        #     for propagated_set in list_propagated_sets:
        #         propagated_set.proposition_holder.add_propositions(set_propositions, "TRAFFIC_STATUS")

        return list_propagated_sets

    def label_in_conflict_area_propositions(self, step,
                                            list_propagated_sets: Union[List[SemanticReachNode], List[pycrreachs.SemanticReachNode]]) \
            -> Union[List[SemanticReachNode], List[pycrreachs.SemanticReachNode]]:
        """
        Labels propagated sets with propositions related to conflict status between them and the vehicles.

        Note: this relationship is asymmetric, refer to Sebastian's intersection traffic rule paper for definition.
        A lanelet is examined against a list of lanelets of the lane/route of the other object.
        """
        # examine if the propagated set is conflicting with the vehicles
        for propagated_set in list_propagated_sets:
            for vehicle in self.list_vehicles:
                # if not vehicle.behind_node_at_step(step, base_set):
                #     continue

                # iterate through lanelet ids of the region
                for id_lanelet_propagated_set in propagated_set.set_ids_lanelets:
                    # iterate through lanelet ids of the lane of the vehicle
                    for id_lanelet_lane_vehicle in vehicle.lane.list_ids_lanelets:
                        # use cached results
                        if id_lanelet_lane_vehicle in \
                                self.dict_id_lanelet_to_set_ids_lanelets_intersecting[id_lanelet_propagated_set]:
                            propagated_set.proposition_holder.add_propositions({P.in_conflict_with(vehicle.id_vehicle)},
                                                                               PG.TRAFFIC_STATUS)

        # examine if the vehicles are in conflict with the propagated set
        for propagated_set in list_propagated_sets:
            if isinstance(propagated_set, SemanticReachNode):
                p_lon_min = propagated_set.p_lon_min
            else:
                p_lon_min = propagated_set.p_lon_min()

            for vehicle in self.list_vehicles:
                try:
                    p_lon_min_propagated_set = p_lon_min - self.config.vehicle.ego.radius_inflation
                    p_lon_ref_max_vehicle = vehicle.dict_step_to_state_lon_ref[step].s + vehicle.shape.length / 2

                except (AttributeError, KeyError):
                    continue

                # propagated set is in front of the vehicle along the reference path
                if p_lon_min_propagated_set > p_lon_ref_max_vehicle:
                    continue

                # iterate through lanelet ids of the route
                for id_lanelet_route in self.config.planning.route.list_ids_lanelets:
                    # iterate through lanelet ids of the vehicle
                    for id_lanelet_vehicle in vehicle.lanelet_ids_at_step(step):
                        if id_lanelet_vehicle in \
                                self.dict_id_lanelet_to_set_ids_lanelets_intersecting[id_lanelet_route]:
                            propagated_set.proposition_holder.add_propositions({P.in_conflict_by(vehicle.id_vehicle)},
                                                                               PG.VEHICLE)

        return list_propagated_sets

    def label_causes_braking_propositions(self, step: int,
                                          list_propagated_sets: Union[List[SemanticReachNode], List[pycrreachs.SemanticReachNode]]) \
            -> Union[List[SemanticReachNode], List[pycrreachs.SemanticReachNode]]:
        """
        Labels propagated sets with propositions related to causes braking to other vehicles.
        """
        # only compute if there is an intersection
        if not self.config.semantic_model.incoming_element_route:
            return list_propagated_sets

        for propagated_set in list_propagated_sets:
            # todo: this should only be computed for vehicles entering an intersection from other directions
            for vehicle in self.list_vehicles:
                if vehicle.braking_caused_by_node_at_step(step, propagated_set):
                    propagated_set.proposition_holder.add_propositions({P.causes_braking_for(vehicle.id_vehicle)},
                                                                       PG.TRAFFIC_STATUS)

        return list_propagated_sets

    def split_wrt_regions(self, step: int, reachable_set: SemanticReachNode) -> List[SemanticReachNode]:
        """
        Splits a reachable set w.r.t lanelet regions.

        Steps:
            1. Intersect reachable set in the position domain with lanelet regions
            2. Over-approximate and restore to axis-aligned rectangles
        """
        list_sets_split = []
        # iterate through regions intersecting with the reachable set
        for region in self.list_regions:
            # first compute intersection with bounding box
            # --> exact intersection is more expensive, so we only want to compute it if necessary?
            # there is no possibility of intersection
            if not region.intersects(reachable_set.position_rectangle.bounds, coordinate_system="CVLN"):
                continue

            # there is a possibility of intersection
            # TODO: Find out, why there was a try-except for Exception here
            polygon_intersection = region.polygon_cvln.intersection(reachable_set.position_rectangle)

            # empty intersection
            if polygon_intersection.is_empty:
                continue

            # over-approximate by restoring the intersected polygon to axis-aligned rectangle
            bounds_polygon_intersection = polygon_intersection.bounds

            # clone the propagated set and split in the position domain, update the propositions
            # TODO: Find out, why there was a try-except for AttributeError here
            reachable_set_new = reachable_set.clone()
            reachable_set_new.intersect_in_position_domain(*bounds_polygon_intersection)
            reachable_set_new = self.update_propositions_with_region(reachable_set_new, region, step)

            list_sets_split.append(reachable_set_new)

        return list_sets_split

    def update_propositions_with_region(self, propagated_set: Union[SemanticReachNode, pycrreachs.SemanticReachNode],
                                        region: Union[Region, pycrreachs.Region], step: int):
        """
        Updates the propositions of the propagated set with the proposition of the lanelet region.

        Lanelet transition proposition are considered to be temporary since they are only used for TPL compliance
        checking and should be omitted during merging of the propagated sets.
        """
        # add propositions of the region to persistent propositions of the propagated set
        if isinstance(region, Region):
            dict_relevant = region.dict_group_to_propositions_at_step(step)

        else:
            dict_relevant = region.map_group_to_propositions_at_step(step)

        for group in dict_relevant:
            set_propositions = dict_relevant[group]
            propagated_set.proposition_holder.add_propositions(set_propositions, group)

        # add lanelet ids of the region to propagated set
        if isinstance(propagated_set, SemanticReachNode):
            propagated_set.set_ids_lanelets.update(region.set_ids_lanelets)

        else:
            propagated_set.add_lanelet_ids(region.set_ids_lanelets)

        # add lanelet transition as temporary propositions
        set_propositions = self.obtain_lanelet_transition_propositions(propagated_set)
        propagated_set.proposition_holder.add_propositions(set_propositions, PG.TEMPORARY)

        return propagated_set

    def split_wrt_position_intervals(self, step: int,
                                      reachable_set: SemanticReachNode) -> List[SemanticReachNode]:
        """
        Splits the reachable set w.r.t position intervals.
        """

        list_intervals_lon: List[PositionInterval] = self.dict_step_to_position_intervals[step]["lon"]
        list_intervals_lat: List[PositionInterval] = self.dict_step_to_position_intervals[step]["lat"]

        list_reachable_sets_split_lon = []
        for interval_lon in list_intervals_lon:
            # propagated set intersects with the longitudinal interval
            if interval_lon.intersects(reachable_set.p_lon_min, reachable_set.p_lon_max):
                propagated_set_split = reach_operation.split_reach_node_to_interval(reachable_set, interval_lon, "lon")
                if propagated_set_split:
                    list_reachable_sets_split_lon.append(propagated_set_split)

            # early termination, since the rest of intervals will definitely not intersect with the base set
            elif interval_lon.p_min > reachable_set.polygon_lon.p_max:
                break

        list_reachable_sets_split = []
        for propagated in list_reachable_sets_split_lon:
            for interval_lat in list_intervals_lat:
                # propagated set intersects with the lateral interval
                if interval_lat.intersects(propagated.p_lat_min, propagated.p_lat_max):
                    propagated_set_split = reach_operation.split_reach_node_to_interval(propagated, interval_lat, "lat")
                    if propagated_set_split:
                        list_reachable_sets_split.append(propagated_set_split)

                # early termination, since the rest of intervals will definitely not intersect with the base set
                elif interval_lat.p_min > propagated.polygon_lat.p_max:
                    break

        return list_reachable_sets_split

    @staticmethod
    def obtain_lanelet_transition_propositions(propagated_set: Union[SemanticReachNode, pycrreachs.SemanticReachNode]):
        """
        Returns the set of lanelet transition propositions.
        """
        set_propositions = set()
        # retrieve lanelet propositions from the source
        if isinstance(propagated_set, SemanticReachNode):
            set_propositions_position_source = \
                propagated_set.source_propagation.proposition_holder.propositions_in_group(group=PG.POSITION)

        else:
            set_propositions_position_source = \
                propagated_set.vec_nodes_source[0].proposition_holder.propositions_in_group(PG.POSITION)

        set_ids_lanelets_source = {int(proposition.split("_")[1]) for proposition in set_propositions_position_source
                                   if P.in_lanelet() in proposition}

        # generate lanelet transition propositions
        for id_lanelet_source in set_ids_lanelets_source:
            for id_lanelet_base_set in propagated_set.set_ids_lanelets:
                if id_lanelet_source != id_lanelet_base_set:
                    set_propositions.add(P.lanelet_transition(id_lanelet_source, id_lanelet_base_set))

        return set_propositions

    @staticmethod
    def discard_colliding_nodes(list_propagated_set: List[SemanticReachNode]) -> List[SemanticReachNode]:
        """
        Returns a list of propagated sets that do not collide with vehicles.
        """
        list_nodes_keep = []

        for propagated_set in list_propagated_set:
            colliding = False
            set_propositions = propagated_set.set_propositions()
            for proposition in set_propositions:
                # check if it is aligned with and besides a vehicle
                if P.aligned_with() in proposition:
                    id_vehicle = int(proposition.split("_")[1][1:])

                    if P.beside(id_vehicle) in set_propositions:
                        colliding = True
                        break

            if not colliding:
                list_nodes_keep.append(propagated_set)

        return list_nodes_keep

    @staticmethod
    def call_python_dummy(step, node):
        """
        Dummy function to be called from C++.
        """
        return node
