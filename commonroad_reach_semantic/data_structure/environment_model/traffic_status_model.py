import itertools
from collections import defaultdict
from typing import Dict, Set

from commonroad_reach_semantic.data_structure.config.semantic_configuration import SemanticConfiguration
from commonroad_reach_semantic.data_structure.environment_model.lanelet_model import LaneletModel
from commonroad_reach_semantic.data_structure.environment_model.vehicle_model import VehicleModel
from commonroad_reach_semantic.data_structure.rule.proposition import Proposition as Prop


class TrafficStatusModel:
    """Computes and stores semantic information related to traffic status in the scenario."""

    config: SemanticConfiguration
    lanelet_model: LaneletModel
    vehicle_model: VehicleModel
    step_start: int
    step_end: int
    dict_step_to_traffic_status_propositions: Dict[int, Set[str]]

    def __init__(self, config: SemanticConfiguration, lanelet_model: LaneletModel, vehicle_model: VehicleModel) -> None:
        self.config = config
        self.lanelet_model = lanelet_model
        self.vehicle_model = vehicle_model
        self.step_start = self.config.planning.step_start
        self.step_end = self.step_start + self.config.planning.steps_computation

        self.dict_step_to_traffic_status_propositions = dict()

        self._determine_traffic_status_propositions()

    def _determine_traffic_status_propositions(self) -> None:
        """
        Determines propositions for the general traffic status.
        """
        dict_step_to_traffic_status_propositions: Dict[int, Set[str]] = defaultdict(set)

        # extract propositions indicating a vehicle is within an intersection
        for id_lanelet in self.lanelet_model.set_ids_lanelets_in_intersections:
            lanelet = self.config.scenario.lanelet_network.find_lanelet_by_id(id_lanelet)

            for step in range(self.step_start, self.step_end + 1):
                for id_obstacle in lanelet.dynamic_obstacle_by_time_step(step):
                    dict_step_to_traffic_status_propositions[step].add(Prop.in_intersection(id_obstacle))

        for vehicle, step in itertools.product(self.vehicle_model.list_vehicles,
                                               range(self.step_start, self.step_end + 1)):
            lanelet_ids_at_step = vehicle.lanelet_ids_at_step(step)

            # extract propositions indicating a vehicle is in its outgoing lanelet
            if vehicle.set_ids_lanelets_successor_incoming.intersection(lanelet_ids_at_step):
                dict_step_to_traffic_status_propositions[step].add(
                    Prop.in_direction_successor(vehicle.type_outgoing, vehicle.id_vehicle))

            # extract propositions indicating a vehicle is in a specific lanelet
            dict_step_to_traffic_status_propositions[step].update(
                Prop.vehicle_in_lanelet(vehicle.id_vehicle, l_id) for l_id in lanelet_ids_at_step
            )

            # extract propositions indicating a vehicle is on the main carriageway
            if not self.lanelet_model.main_carriageway_lanelet_ids.isdisjoint(lanelet_ids_at_step):
                dict_step_to_traffic_status_propositions[step].add(Prop.vehicle_on_main_carriageway(vehicle.id_vehicle))

            # extract propositions indicating a vehicle is on an access ramp
            if not self.lanelet_model.access_ramp_lanelet_ids.isdisjoint(lanelet_ids_at_step):
                dict_step_to_traffic_status_propositions[step].add(
                    Prop.vehicle_on_access_ramp(vehicle.id_vehicle))

        self.dict_step_to_traffic_status_propositions = dict_step_to_traffic_status_propositions
