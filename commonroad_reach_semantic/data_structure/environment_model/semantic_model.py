import logging
from typing import List, Dict

import commonroad_reach.utility.logger as util_logger

from commonroad_reach_semantic.data_structure.config.semantic_configuration import SemanticConfiguration
from commonroad_reach_semantic.data_structure.environment_model.lanelet_model import LaneletModel
from commonroad_reach_semantic.data_structure.environment_model.region_model import RegionModel
from commonroad_reach_semantic.data_structure.environment_model.traffic_status_model import TrafficStatusModel
from commonroad_reach_semantic.data_structure.environment_model.vehicle_model import VehicleModel
from commonroad_reach_semantic.data_structure.reach.semantic_reach_node import SemanticReachNode
from commonroad_reach_semantic.data_structure.rule.proposition import Proposition as P

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

        self.lanelet_model = LaneletModel(self.config)
        self.vehicle_model = VehicleModel(self.config, self.lanelet_model)
        self.region_model = RegionModel(self.config, self.lanelet_model, self.vehicle_model)
        self.traffic_status_model = TrafficStatusModel(self.config, self.lanelet_model, self.vehicle_model)

        logger.info("SemanticModel created.")
        self.print_summary()

    def print_summary(self):
        string = "# ========= Model Summary ========= #\n"
        string += f"#\tLanes: {len(self.lanelet_model.road_network.list_lanes)}\n"
        string += f"#\tVehicles: {len(self.vehicle_model.list_vehicles)}\n"
        string += f"#\tRegions: {len(self.region_model.list_regions)}\n"
        string += "# ================================= #"

        for line in string.split("\n"):
            util_logger.print_and_log_info(logger, line)

    def determine_traffic_priorities(self, dict_traffic_sign_to_priorities: Dict):
        """
        Determines the traffic priorities for regions and vehicles.
        """
        if self.config.semantic_model.incoming_element_route:
            # vehicles
            self.vehicle_model.determine_traffic_priorities(dict_traffic_sign_to_priorities)

            # lanelet regions
            self.region_model.determine_traffic_priorities(dict_traffic_sign_to_priorities)

            logger.info("Traffic priorities determined.")

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
