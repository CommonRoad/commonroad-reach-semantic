import logging
import warnings
from collections import defaultdict

import numpy as np
from commonroad.scenario.lanelet import LaneletNetwork

from commonroad_reach_semantic.data_structure.config.semantic_configuration import SemanticConfiguration
from commonroad_reach_semantic.data_structure.environment_model.road_network import RoadNetwork
from commonroad_reach_semantic.utility import reach_operation

logger = logging.getLogger(__name__)


class LaneletModel:
    def __init__(self, config: SemanticConfiguration) -> None:
        self.config = config
        self.set_lanelets_route_related = set()
        self.set_ids_lanelets_same_direction = set()
        self.set_ids_lanelets_opposite_direction = set()
        self.set_ids_lanelets_in_intersections = set()
        self.local_lanelet_network = None
        self.road_network = None
        self.dict_id_lanelet_to_lanelet = dict()
        self.dict_id_lanelet_to_set_ids_lanelets_intersecting = defaultdict(set)

        self._create_local_lanelet_network_and_road_network()

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
