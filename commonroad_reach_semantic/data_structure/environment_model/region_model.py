import itertools
import logging
from typing import List, Dict, Set, Optional, FrozenSet

from commonroad.scenario.traffic_sign import TrafficLightState, TrafficLightDirection

import commonroad_reach_semantic.utility.region as util_region
from commonroad_reach_semantic.data_structure.config.outgoing_direction import OutgoingDirection
from commonroad_reach_semantic.data_structure.config.semantic_configuration import SemanticConfiguration
from commonroad_reach_semantic.data_structure.environment_model.lanelet_model import LaneletModel
from commonroad_reach_semantic.data_structure.environment_model.region import Region
from commonroad_reach_semantic.data_structure.environment_model.vehicle_model import VehicleModel
from commonroad_reach_semantic.data_structure.rule.proposition import Proposition as Prop
from commonroad_reach_semantic.data_structure.rule.proposition import PropositionGroup as PropGroup

logger = logging.getLogger(__name__)


class RegionModel:
    """Computes and stores semantic information related to lanelet regions."""

    config: SemanticConfiguration
    lanelet_model: LaneletModel
    vehicle_model: VehicleModel
    step_start: int
    step_end: int
    list_regions: List[Region]
    _lanelet_ids_to_region: Dict[FrozenSet[int], Region]

    def __init__(self, config: SemanticConfiguration, lanelet_model: LaneletModel, vehicle_model: VehicleModel) -> None:
        self.config = config
        self.lanelet_model = lanelet_model
        self.vehicle_model = vehicle_model
        self.step_start = self.config.planning.step_start
        self.step_end = self.step_start + self.config.planning.steps_computation

        self.list_regions = list()
        self._lanelet_ids_to_region = dict()

        self._create_lanelet_regions()

        for region in self.list_regions:
            self._lanelet_ids_to_region[frozenset(region.set_ids_lanelets)] = region

        self._determine_propositions()

    def find_region_by_lanelet_ids(self, lanelet_ids: Set[int]) -> Optional[Region]:
        """Finds a region by its lanelet IDs.

        :param lanelet_ids: lanelet IDs to look for
        :return: region if found, None otherwise
        """
        return self._lanelet_ids_to_region.get(frozenset(lanelet_ids))

    def determine_traffic_priorities(self, dict_traffic_sign_to_priorities: Dict):
        """
        Determines the traffic priorities of the regions in the scenario.
        """
        for region in self.list_regions:
            region.determine_priorities(dict_traffic_sign_to_priorities)
            region.examine_priorities_against_vehicles(self.vehicle_model.list_vehicles)

    def _create_lanelet_regions(self) -> None:
        """
        Constructs lanelet regions both in the Cartesian and curvilinear coordinate systems

        A lanelet region is a polygon which encloses all positions within the lanelet. The reachable sets are later
        cut down to lanelet regions for determining their position propositions.
        """
        Region.initialize(self.config, self.lanelet_model.road_network, self.lanelet_model.set_lanelets_route_related)
        set_tuples_ids_lanelets_intersecting = \
            util_region.detect_intersecting_lanelets(self.lanelet_model.set_lanelets_route_related)

        list_regions = util_region.construct_regions_for_intersecting_lanelets(set_tuples_ids_lanelets_intersecting)

        # non-intersecting regions only appear if the shape of the ego vehicle is not considered
        if not self.config.semantic_model.consider_ego_shape:
            list_regions += \
                util_region.construct_regions_for_nonintersecting_lanelets(set_tuples_ids_lanelets_intersecting)

        self.list_regions = list_regions

        logger.info("Lanelet regions created.")

    def _determine_propositions(self) -> None:
        """
        Determines relevant propositions.
        """
        self._label_region_with_time_invariant_propositions()
        self._label_region_with_time_variant_propositions()

        logger.info("Propositions determined.")

    def _label_region_with_time_invariant_propositions(self) -> None:
        """
        Labels regions with time invariant propositions.
        """
        self._label_driving_direction_propositions()
        self._label_lanelet_type_propositions()
        self._label_region_vehicle_intersection_incoming_propositions()
        self._label_region_vehicle_same_lane_propositions()

    def _label_driving_direction_propositions(self) -> None:
        """
        Labels regions with propositions related to driving directions.
        """
        for region in self.list_regions:
            if not region.set_ids_lanelets.intersection(self.lanelet_model.set_ids_lanelets_opposite_direction):
                region.proposition_holder.add_proposition(Prop.same_driving_direction(), PropGroup.TRAFFIC_SIGN)

    def _label_lanelet_type_propositions(self) -> None:
        """
        Labels regions with propositions related to lanelet types.
        """
        # in intersection
        for region in self.list_regions:
            if region.set_ids_lanelets.intersection(self.lanelet_model.set_ids_lanelets_in_intersections):
                region.proposition_holder.add_proposition(Prop.in_intersection(), PropGroup.POSITION)

        # in successor lanelets of the incoming element
        direction_outgoing = self.config.semantic_model.direction_outgoing
        match direction_outgoing:
            case OutgoingDirection.LEFT:
                incoming_successors = self.config.semantic_model.incoming_element_route.successors_left
            case OutgoingDirection.STRAIGHT:
                incoming_successors = self.config.semantic_model.incoming_element_route.successors_straight
            case OutgoingDirection.RIGHT:
                incoming_successors = self.config.semantic_model.incoming_element_route.successors_right
            case _:
                return None

        for region in self.list_regions:
            if region.set_ids_lanelets.intersection(incoming_successors):
                region.proposition_holder.add_proposition(Prop.in_direction_successor(direction_outgoing),
                                                          PropGroup.POSITION)

    def _label_region_vehicle_intersection_incoming_propositions(self) -> None:
        """
        Updates the intersection incoming relations between the region and vehicles over time.

        This definition is different from the one in Sebastian's IV2022 paper. The proposition is propagated to
        successor lanelets of the incoming lanelets so that it is still present even after entering the intersection.
        """
        incoming_element_route = Region.incoming_element_route
        match self.config.semantic_model.direction_outgoing:
            case OutgoingDirection.LEFT:
                incoming_element_route_successors = incoming_element_route.successors_left
            case OutgoingDirection.STRAIGHT:
                incoming_element_route_successors = incoming_element_route.successors_straight
            case OutgoingDirection.RIGHT:
                incoming_element_route_successors = incoming_element_route.successors_right
            case _:
                return None
        set_ids_lanelets_effective = incoming_element_route.incoming_lanelets.union(incoming_element_route_successors)

        for vehicle in self.vehicle_model.list_vehicles:
            incoming_element_vehicle = vehicle.incoming_element
            if not incoming_element_vehicle:
                continue

            # if the route's incoming element is left of vehicle's incoming element, propagate this to the incoming and
            # corresponding successor lanelets of the incoming element.
            if incoming_element_route.left_of == incoming_element_vehicle.incoming_id:
                for region in self.list_regions:
                    if region.set_ids_lanelets.intersection(set_ids_lanelets_effective):
                        region.proposition_holder.add_proposition(Prop.intersection_left_of(vehicle.id_vehicle),
                                                                  PropGroup.INTERSECTION)

    def _label_region_vehicle_same_lane_propositions(self) -> None:
        """
        Updates the lane relation between the regions and the vehicles.
        """
        for region, vehicle in itertools.product(self.list_regions, self.vehicle_model.list_vehicles):
            lane_vehicle = vehicle.lane
            if lane_vehicle in region.set_lanes:
                region.proposition_holder.add_proposition(Prop.in_same_lane(vehicle.id_vehicle), PropGroup.VEHICLE)

    def _label_region_with_time_variant_propositions(self) -> None:
        """
        Updates time variant propositions of the region.
        """
        self._label_region_vehicle_intersection_oncoming_propositions()
        self._label_region_vehicle_outgoing_propositions()
        self._label_traffic_light_status_propositions()

    def _label_region_vehicle_intersection_oncoming_propositions(self) -> None:
        """
        Updates the intersection oncoming relations between the region and vehicles over time.
        """
        # examine if vehicle is on oncoming of the region
        for region in self.list_regions:
            incoming_region = region.incoming_element
            if not incoming_region:
                continue

            for vehicle, step in itertools.product(self.vehicle_model.list_vehicles,
                                                   range(self.step_start, self.step_end + 1)):
                list_ids_lanelets_vehicle_at_step = vehicle.lanelet_ids_at_step(step)

                if region.set_ids_lanelets_oncoming.intersection(list_ids_lanelets_vehicle_at_step):
                    region.proposition_holder.add_proposition(Prop.on_oncoming(vehicle.id_vehicle),
                                                              PropGroup.INTERSECTION, step)

        # examine if the region is on oncoming of the vehicle
        for region, vehicle in itertools.product(self.list_regions, self.vehicle_model.list_vehicles):
            if region.set_ids_lanelets.intersection(vehicle.set_ids_lanelets_oncoming):
                region.proposition_holder.add_proposition(Prop.on_oncoming_of(vehicle.id_vehicle),
                                                          PropGroup.INTERSECTION)

    def _label_region_vehicle_outgoing_propositions(self) -> None:
        """
        Updates the intersection outgoing relations between the region the vehicles over time.
        """
        for region, vehicle, step in itertools.product(self.list_regions, self.vehicle_model.list_vehicles,
                                                       range(self.step_start, self.step_end + 1)):
            for dir_region, dir_vehicle in itertools.product(OutgoingDirection, repeat=2):
                if region.outgoing_lanelet_ids(dir_region).intersection(vehicle.outgoings_at_step(step, dir_vehicle)):
                    proposition = Prop.region_out_same_as_vehicle_out(dir_region, dir_vehicle, vehicle.id_vehicle)
                    region.proposition_holder.add_proposition(proposition, PropGroup.PRIORITY, step)

    def _label_traffic_light_status_propositions(self) -> None:
        """
        Updates traffic light status of the region.
        """
        props = [
            Prop.at_red_left_traffic_light(),
            Prop.at_red_straight_traffic_light(),
            Prop.at_red_right_traffic_light()
        ]
        traffic_light_directions = [
            # left
            [TrafficLightDirection.LEFT, TrafficLightDirection.LEFT_STRAIGHT,
             TrafficLightDirection.LEFT_RIGHT, TrafficLightDirection.ALL],
            # straight
            [TrafficLightDirection.STRAIGHT, TrafficLightDirection.LEFT_STRAIGHT,
             TrafficLightDirection.STRAIGHT_RIGHT, TrafficLightDirection.ALL],
            # right
            [TrafficLightDirection.RIGHT, TrafficLightDirection.LEFT_RIGHT,
             TrafficLightDirection.STRAIGHT_RIGHT, TrafficLightDirection.ALL]
        ]

        for region, step in itertools.product(self.list_regions, range(self.step_start, self.step_end + 1)):
            for traffic_light in region.set_traffic_lights_active:
                state_light = traffic_light.get_state_at_time_step(step)

                if state_light in [TrafficLightState.RED, TrafficLightState.RED_YELLOW]:
                    for prop, directions in zip(props, traffic_light_directions):
                        if traffic_light.direction in directions:
                            region.proposition_holder.add_proposition(prop, PropGroup.TRAFFIC_LIGHT, step)
