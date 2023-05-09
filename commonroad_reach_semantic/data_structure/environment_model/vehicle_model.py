import itertools
import logging
from collections import defaultdict
from functools import lru_cache
from typing import List, Union, Set, Dict, Optional

import numpy as np
from commonroad.scenario.lanelet import LaneletType
from commonroad.scenario.obstacle import DynamicObstacle, Obstacle, StaticObstacle, EnvironmentObstacle, PhantomObstacle

from commonroad_reach_semantic.data_structure.config.semantic_configuration import SemanticConfiguration
from commonroad_reach_semantic.data_structure.environment_model.lanelet_model import LaneletModel
from commonroad_reach_semantic.data_structure.environment_model.position_interval import PositionInterval
from commonroad_reach_semantic.data_structure.environment_model.vehicle import Vehicle

logger = logging.getLogger(__name__)


class VehicleModel:
    """Computes and stores semantic information related to vehicles in the scenario."""

    config: SemanticConfiguration
    lanelet_model: LaneletModel
    step_start: int
    step_end: int
    list_vehicles: List[Vehicle]
    set_ids_vehicles_entering_intersection: Set[int]
    dict_step_to_position_intervals: Dict[int, Dict[str, List[PositionInterval]]]

    def __init__(self, config: SemanticConfiguration, lanelet_model: LaneletModel) -> None:
        self.config = config
        self.lanelet_model = lanelet_model
        self.step_start = self.config.planning.step_start
        self.step_end = self.step_start + self.config.planning.steps_computation

        self.list_vehicles: List[Vehicle] = list()
        self.set_ids_vehicles_entering_intersection = set()
        self.dict_sonia_prediction = defaultdict(dict)

        self.dict_step_to_position_intervals = dict()

        self._create_vehicles()
        self._create_position_intervals()

    def determine_traffic_priorities(self, dict_traffic_sign_to_priorities: Dict):
        """
        Determines the traffic priorities of the vehicles in the scenario.
        """
        for vehicle in self.list_vehicles:
            vehicle.determine_priorities(dict_traffic_sign_to_priorities)

    @lru_cache(maxsize=None)
    def find_vehicle_by_id(self, id_vehicle: int) -> Optional[Vehicle]:
        """
        Returns the vehicle object with the given id.
        """
        for vehicle in self.list_vehicles:
            if vehicle.id_vehicle == id_vehicle:
                return vehicle
        return None

    def _create_vehicles(self) -> None:
        """
        Creates vehicle objects from relevant obstacles in the scenario.
        """
        Vehicle.initialize(self.config, self.lanelet_model.road_network)

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

    def _retrieve_relevant_obstacles(self, fov=200, bound_with_circle=True) -> List[
        Union[Obstacle, StaticObstacle, DynamicObstacle, EnvironmentObstacle, PhantomObstacle]]:
        """
        Returns a list of obstacles in the scenario to be considered in the computation.

        Computes a circle with the initial position as the center, and the fov of the ego vehicle as the radius.
        The vehicles within this radius are deemed as relevant obstacles.
        """
        if bound_with_circle:
            # return vehicles within the fov of the ego vehicle
            return [
                obs for obs in self.config.scenario.obstacles
                # include if distance between the initial position of ego and other vehicles is smaller than fov
                if
                np.linalg.norm(obs.initial_state.position - self.config.planning_problem.initial_state.position) <= fov
            ]
        else:
            # return all obstacles in the scenario
            return self.config.scenario.obstacles

    def _add_obstacles_to_lanelets(self, list_obstacles: List[
        Union[Obstacle, StaticObstacle, DynamicObstacle, EnvironmentObstacle, PhantomObstacle]]) -> None:
        """
        Adds obstacles to lanelets.

        An obstacle is added to a lanelet if its occupancy in the future time steps intersects with the lanelet.
        """
        for obstacle in list_obstacles:
            for lanelet in self.lanelet_model.local_lanelet_network.lanelets:
                polygon_lanelet = lanelet.polygon.shapely_object

                for step in range(self.step_start, self.step_end + 1):
                    time_step = step * round(self.config.planning.dt * 10)
                    occupancy = obstacle.occupancy_at_time(time_step)
                    if occupancy and occupancy.shape.shapely_object.intersects(polygon_lanelet):
                        if isinstance(obstacle, DynamicObstacle):
                            lanelet.add_dynamic_obstacle_to_lanelet(obstacle.obstacle_id, time_step)
                        else:
                            lanelet.add_static_obstacle_to_lanelet(obstacle.obstacle_id)
                            break

    def _create_vehicles_from_obstacles(self, list_obstacles: List[
        Union[Obstacle, StaticObstacle, DynamicObstacle, EnvironmentObstacle, PhantomObstacle]]) -> List[Vehicle]:
        """
        Creates a list of vehicle objects from the given list of obstacles.
        """
        return [
            vehicle for obstacle in list_obstacles
            if (vehicle := Vehicle.create_vehicle_from_obstacle(obstacle, self.dict_sonia_prediction))
        ]

    def _retrieve_vehicles_entering_intersection(self) -> Set[int]:
        """
        Returns set of ids of vehicles entering intersection
        """
        # alternatively, one can also check the lanelets a vehicle occupies during the planning horizon
        return {
            vehicle.id_vehicle for vehicle in self.list_vehicles
            if LaneletType.INTERSECTION in (
                self.lanelet_model.local_lanelet_network.find_lanelet_by_id(id_lanelet).lanelet_type
                for id_lanelet in vehicle.lane.list_ids_lanelets
            )
        }

    def _create_position_intervals(self) -> None:
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
        for step in range(self.step_start, self.step_end + 1):
            list_intervals_lon = [interval_lon_initial.clone()]
            list_intervals_lat = [interval_lat_initial.clone()]

            for vehicle in self.list_vehicles:
                list_intervals_lon = list(itertools.chain.from_iterable(
                    interval_lon.split_with_respect_to_vehicle(step, vehicle, length_ego / 2, "lon")
                    for interval_lon in list_intervals_lon
                ))
                list_intervals_lat = list(itertools.chain.from_iterable(
                    interval_lat.split_with_respect_to_vehicle(step, vehicle, width_ego / 2, "lat")
                    for interval_lat in list_intervals_lat
                ))

            # sort longitudinal and lateral position intervals
            list_intervals_lon.sort(key=lambda interval: interval.p_min)
            list_intervals_lat.sort(key=lambda interval: interval.p_min)
            self.dict_step_to_position_intervals[step] = {
                "lon": list_intervals_lon,
                "lat": list_intervals_lat,
            }

        logger.info("Position intervals created.")
