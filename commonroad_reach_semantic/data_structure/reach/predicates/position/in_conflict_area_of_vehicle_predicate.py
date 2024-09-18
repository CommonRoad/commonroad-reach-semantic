import time
from typing import List, Optional, Set

import shapely
from commonroad.common.common_lanelet import LaneletType
from commonroad.scenario.lanelet import Lanelet
from commonroad_reach.data_structure.reach.reach_node import ReachNode
from commonroad_reach.data_structure.reach.reach_polygon import ReachPolygon

import commonroad_reach_semantic.data_structure.reach.predicates.predicate as predicate
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.rule.proposition import Proposition as Prop
import commonroad_reach.utility.coordinate_system as util_cosy

class InConflictAreaOfVehiclePredicate(predicate.Predicate):
    _cached_conflict_region_enl_clcs_polygon = None  # Cache storage

    def __init__(self, vehicle_id: int, negated: bool):
        super().__init__(negated)
        self.vehicle_id = vehicle_id
        self.needs_lanelets = True

    def to_proposition(self) -> str:
        return Prop.in_conflict_with(self.vehicle_id)

    @predicate.needs_lanelets_set
    def _restrict_reach_node_mandatory(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel,
                                       node_lanelet_ids: Set[int]) -> List[ReachNode]:

        # Retrieve the vehicle object using its ID
        vehicle = semantic_model.vehicle_model.find_vehicle_by_id(self.vehicle_id)

        # Get the lanelet IDs for the ego vehicle and the other vehicle
        lanelets_dir_ego = set(semantic_model.config.planning.route.lanelet_ids)
        lanelets_dir_other = set(vehicle.lanelets_dir)

        # Determine the lanelets that are in the other vehicle's path but not in the ego vehicle's path
        disjoint_other = list(lanelets_dir_other - lanelets_dir_ego)
        # todo: need to be fixed
        if not disjoint_other or node_lanelet_ids.isdisjoint(disjoint_other):
            return []
        else:
            return [reach_node]

    @predicate.needs_lanelets_set
    def _restrict_reach_node_forbidden(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel,
                                       node_lanelet_ids: Set[int]) -> List[ReachNode]:
        # Retrieve the vehicle object using its ID
        vehicle = semantic_model.vehicle_model.find_vehicle_by_id(self.vehicle_id)

        # Get the lanelet IDs for the ego vehicle and the other vehicle
        lanelets_dir_ego = set(semantic_model.config.planning.route.lanelet_ids)
        lanelets_dir_other = set(vehicle.lanelets_dir)

        lanelet_network = semantic_model.config.scenario.lanelet_network
        lanelets_ego_intersection = self.get_intersection_lanelets(lanelet_network, lanelets_dir_ego)
        lanelets_other_intersection = self.get_intersection_lanelets(lanelet_network, lanelets_dir_other)

        # if set(lanelets_ego_intersection).isdisjoint(set(lanelets_other_intersection)):
        #     return [reach_node]

        # Only compute conflict_region_enl_clcs_polygon once and reuse it
        if self._cached_conflict_region_enl_clcs_polygon is None:
            time_start = time.time()

            # Calculate the union of the ego lanelets and other vehicle's lanelets at intersections
            ego_intersection_region = self.get_lanelet_union(lanelets_ego_intersection)
            other_intersection_region = self.get_lanelet_union(lanelets_other_intersection)

            # Find the intersection of the two regions
            conflict_region = ego_intersection_region.intersection(other_intersection_region)

            vehicle_length_add = semantic_model.config.vehicle.ego.radius_disc * 2

            conflict_region_enlarged = shapely.offset_curve(
                conflict_region, vehicle_length_add
            )
            conflict_region_enl_polygon = shapely.Polygon(conflict_region_enlarged)

            conflict_region_enl_clcs = util_cosy.convert_to_curvilinear_vertices(
                conflict_region_enl_polygon.exterior.coords, semantic_model.config.planning.CLCS
            )
            # Assuming convert_to_curvilinear_vertices returns coordinates, convert them back to a polygon
            self._cached_conflict_region_enl_clcs_polygon = shapely.Polygon(conflict_region_enl_clcs)
            print(f"Time used for conflict region enlargement: {time.time() - time_start:.5f}s")

        # Use the cached polygon
        conflict_region_enl_clcs_polygon = self._cached_conflict_region_enl_clcs_polygon

        if not hasattr(reach_node.position_rectangle, "shapely_object"):
            # todo: error handling
            node_position_rectangle = shapely.Polygon(reach_node.position_rectangle.vertices)
        else:
            node_position_rectangle = reach_node.position_rectangle.shapely_object

        non_conflict_poly = node_position_rectangle - conflict_region_enl_clcs_polygon
        if non_conflict_poly.is_empty:
            return []

        if isinstance(non_conflict_poly, shapely.geometry.MultiPolygon):
            for unit_poly in list(non_conflict_poly.geoms):
                resulting_position_rectangle = ReachPolygon.from_polygon(unit_poly)
                reach_node.intersect_in_position_domain(p_lon_max=resulting_position_rectangle.p_lon_max,
                                                        p_lon_min=resulting_position_rectangle.p_lon_min,
                                                        p_lat_max=resulting_position_rectangle.p_lat_max,
                                                        p_lat_min=resulting_position_rectangle.p_lat_min)

        else:
            resulting_position_rectangle = ReachPolygon.from_polygon(non_conflict_poly)
            reach_node.intersect_in_position_domain(p_lon_max=resulting_position_rectangle.p_lon_max,
                                                    p_lon_min=resulting_position_rectangle.p_lon_min,
                                                    p_lat_max=resulting_position_rectangle.p_lat_max,
                                                    p_lat_min=resulting_position_rectangle.p_lat_min)
        return [reach_node]

    @staticmethod
    def get_intersection_lanelets(lanelet_network, lanelet_ids):
        return [
            lanelet_network.find_lanelet_by_id(lanelet_id)
            for lanelet_id in lanelet_ids
            if LaneletType.INTERSECTION in lanelet_network.find_lanelet_by_id(lanelet_id).lanelet_type
        ]

    @staticmethod
    def get_lanelet_union(lanelets: List[Lanelet]) -> shapely.geometry.base.BaseGeometry:
        # Create a union of all lanelet geometries
        return shapely.unary_union([lanelet.polygon.shapely_object for lanelet in lanelets])
