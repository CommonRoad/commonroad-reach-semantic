import functools
import logging
import warnings
from collections import defaultdict
from typing import Set, Dict, Callable

import numpy as np
from commonroad.scenario.lanelet import LaneletNetwork, Lanelet, LaneletType

from commonroad_reach_semantic.data_structure.config.semantic_configuration import SemanticConfiguration
from commonroad_reach_semantic.data_structure.environment_model.road_network import RoadNetwork
from commonroad_reach_semantic.utility import reach_operation

logger = logging.getLogger(__name__)


class LaneletModel:
    """Computes and stores semantic information related to lanelets in the scenario."""

    config: SemanticConfiguration
    set_lanelets_route_related: Set[Lanelet]
    set_ids_lanelets_same_direction: Set[int]
    set_ids_lanelets_opposite_direction: Set[int]
    set_ids_lanelets_in_intersections: Set[int]
    local_lanelet_network: LaneletNetwork
    road_network: RoadNetwork
    dict_id_lanelet_to_set_ids_lanelets_intersecting: Dict[int, Set[int]]

    def __init__(self, config: SemanticConfiguration) -> None:
        self.config = config
        self.set_lanelets_route_related = set()
        self.set_ids_lanelets_same_direction = set()
        self.set_ids_lanelets_opposite_direction = set()
        self.set_ids_lanelets_in_intersections = set()
        self.local_lanelet_network = None
        self.road_network = None
        self.dict_id_lanelet_to_set_ids_lanelets_intersecting = defaultdict(set)

        self._create_local_lanelet_network_and_road_network()

    def _create_local_lanelet_network_and_road_network(self) -> None:
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
        set_ids_lanelets = set(self.config.planning.route.lanelet_ids)

        # obtain lanelets in the same direction as the route
        self._explore_lanelets(set_ids_lanelets, condition_left=lambda lanelet: lanelet.adj_left_same_direction,
                               condition_right=lambda lanelet: lanelet.adj_right_same_direction)

        set_ids_lanelets_same_direction = set_ids_lanelets.copy()

        # obtain lanelets in both same and opposite directions
        self._explore_lanelets(set_ids_lanelets)

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

    def _explore_lanelets(self, lanelet_ids: Set[int], condition_left: Callable[[Lanelet], bool] = lambda _: True,
                          condition_right: Callable[[Lanelet], bool] = lambda _: True) -> None:
        """Iteratively add all lanelets adjacent to the initial set of lanelets."""
        while True:
            new_lanelet_ids = set()
            for id_lanelet in list(lanelet_ids):
                lanelet = self.config.scenario.lanelet_network.find_lanelet_by_id(id_lanelet)

                # if left lanelet is in the same direction
                if lanelet.adj_left and condition_left(lanelet):
                    new_lanelet_ids.add(lanelet.adj_left)

                # if right lanelet is in the same direction
                if lanelet.adj_right and condition_right(lanelet):
                    new_lanelet_ids.add(lanelet.adj_right)

            # if no new lanelets were added, terminate
            if new_lanelet_ids.issubset(lanelet_ids):
                return

            lanelet_ids.update(new_lanelet_ids)

    def _obtain_lanelets_in_proximity_of_route(self) -> Set[Lanelet]:
        list_lanelets_route = [self.config.scenario.lanelet_network.find_lanelet_by_id(id_lanelet)
                               for id_lanelet in self.config.planning.route.lanelet_ids]
        # get the coordinates of the bounding box
        list_vertices = [vertex for lanelet in list_lanelets_route for vertex in lanelet.center_vertices]

        x_min = min(x for x, y in list_vertices)
        x_max = max(x for x, y in list_vertices)
        y_min = min(y for x, y in list_vertices)
        y_max = max(y for x, y in list_vertices)
        vertex_circle = np.array([(x_max + x_min) / 2.0, (y_max + y_min) / 2.0])
        radius_circle = max(x_max - x_min, y_max - y_min)

        return set(self.config.scenario.lanelet_network.lanelets_in_proximity(vertex_circle, radius_circle))

    def _create_local_lanelet_network(self, set_lanelets: Set[Lanelet]) -> LaneletNetwork:
        """
        Returns a lanelet network with the given set of lanelets.
        """
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            local_lanelet_network = LaneletNetwork.create_from_lanelet_network(self.config.scenario.lanelet_network)

            # add missing intersections property
            for intersection in self.config.scenario.lanelet_network.intersections:
                local_lanelet_network.add_intersection(intersection)

            # First clear and then readd to keep traffic signs and traffic lights from the original network?

            # clear existing lanelets
            for lanelet in local_lanelet_network.lanelets:
                local_lanelet_network.remove_lanelet(lanelet.lanelet_id)

            # add lanelets
            for lanelet in set_lanelets:
                if local_lanelet_network.find_lanelet_by_id(lanelet.lanelet_id) is None:
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

    @property
    @functools.lru_cache(maxsize=1)
    def main_carriageway_lanelet_ids(self) -> Set[int]:
        """Lanelet IDs of the main carriageway."""
        return self._lanelet_ids_by_type(LaneletType.MAIN_CARRIAGE_WAY)

    @property
    @functools.lru_cache(maxsize=1)
    def stop_line_lanelet_ids(self) -> Set[int]:
        """Lanelet IDs of the stop lines."""
        return {
            lanelet.lanelet_id for lanelet in self.local_lanelet_network.lanelets
            if lanelet.stop_line
        }

    @property
    @functools.lru_cache(maxsize=1)
    def access_ramp_lanelet_ids(self) -> Set[int]:
        """Lanelet IDs of the access ramp(s)."""
        return self._lanelet_ids_by_type(LaneletType.ACCESS_RAMP)

    def _lanelet_ids_by_type(self, lanelet_type: LaneletType) -> Set[int]:
        """Lanelet IDs of the given type."""
        return {
            lanelet.lanelet_id for lanelet in self.local_lanelet_network.lanelets
            if lanelet_type in lanelet.lanelet_type
        }

    @property
    @functools.lru_cache(maxsize=1)
    def right_lane_lanelet_ids(self) -> Set[int]:
        """Lanelet IDs of the right lane on the main carriageway."""
        return {
            lanelet.lanelet_id for lanelet in self.local_lanelet_network.lanelets
            if self._is_rightmost_lanelet(lanelet)
        }

    def _is_rightmost_lanelet(self, lanelet: Lanelet) -> bool:
        """Check if lanelet is the rightmost lanelet in the lanelet network."""
        adj_right_not_mcw = LaneletType.MAIN_CARRIAGE_WAY not in self.local_lanelet_network.find_lanelet_by_id(
            lanelet.adj_right).lanelet_type if lanelet.adj_right is not None else True
        return LaneletType.MAIN_CARRIAGE_WAY in lanelet.lanelet_type and adj_right_not_mcw
