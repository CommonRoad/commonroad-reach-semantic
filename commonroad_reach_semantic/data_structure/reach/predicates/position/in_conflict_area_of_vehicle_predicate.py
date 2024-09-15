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

        # Calculate the union of the ego lanelets and other vehicle's lanelets at intersections
        ego_intersection_region = self.get_lanelet_union(lanelets_ego_intersection)
        other_intersection_region = self.get_lanelet_union(lanelets_other_intersection)

        # Find the intersection of the two regions
        conflict_region = ego_intersection_region.intersection(other_intersection_region)
        if conflict_region.is_empty:
            return [reach_node]

        vehicle_length_add = semantic_model.config.vehicle.ego.length / 2
        if semantic_model.config.planning.reference_point == "REAR":
            vehicle_length_add += semantic_model.config.vehicle.ego.wb_rear_axle

        conflict_region_enlarged = shapely.offset_curve(
            conflict_region, vehicle_length_add
        )
        conflict_region_enl_polygon = shapely.Polygon(conflict_region_enlarged)

        conflict_region_enl_clcs = util_cosy.convert_to_curvilinear_vertices(
            conflict_region_enl_polygon.exterior.coords, semantic_model.config.planning.CLCS
        )
        # Assuming convert_to_curvilinear_vertices returns coordinates, convert them back to a polygon
        conflict_region_enl_clcs_polygon = shapely.Polygon(conflict_region_enl_clcs)

        non_conflict_poly = reach_node.position_rectangle.shapely_object - conflict_region_enl_clcs_polygon
        if non_conflict_poly.is_empty:
            return []
        reach_node.position_rectangle = ReachPolygon.from_polygon(non_conflict_poly)
        # import matplotlib.pyplot as plt
        # # Plotting
        #
        # fig, ax = plt.subplots()
        #
        # # Plot the original conflict region (assuming it's a Polygon or MultiPolygon)
        # if conflict_region_enl_clcs_polygon.geom_type == 'Polygon':
        #     x, y = conflict_region_enl_clcs_polygon.exterior.xy
        #     ax.fill(x, y, alpha=0.5, fc='lightblue', label="Original Conflict Region")
        # elif conflict_region_enl_clcs_polygon.geom_type == 'MultiPolygon':
        #     for polygon in conflict_region_enl_clcs_polygon:
        #         x, y = polygon.exterior.xy
        #         ax.fill(x, y, alpha=0.5, fc='lightblue', label="Original Conflict Region")
        #
        # # Plot the enlarged conflict region
        # if conflict_region_enlarged.geom_type == 'Polygon':
        #     x, y = conflict_region_enlarged.exterior.xy
        #     ax.fill(x, y, alpha=0.5, fc='red', label="Enlarged Conflict Region")
        # elif conflict_region_enlarged.geom_type == 'MultiPolygon':
        #     for polygon in conflict_region_enlarged:
        #         x, y = polygon.exterior.xy
        #         ax.fill(x, y, alpha=0.5, fc='red', label="Enlarged Conflict Region")
        #
        # # Plot the position rectangle after difference operation
        # position_rectangle_polygon = reach_node.position_rectangle.shapely_object - conflict_region_enl_clcs_polygon
        # if position_rectangle_polygon.geom_type == 'Polygon':
        #     x, y = position_rectangle_polygon.exterior.xy
        #     ax.fill(x, y, alpha=0.5, fc='green', label="Position Rectangle After Difference")
        # elif position_rectangle_polygon.geom_type == 'MultiPolygon':
        #     for polygon in position_rectangle_polygon:
        #         x, y = polygon.exterior.xy
        #         ax.fill(x, y, alpha=0.5, fc='green', label="Position Rectangle After Difference")
        #
        # # Add labels and legend
        # ax.set_xlabel('X')
        # ax.set_ylabel('Y')
        # ax.legend()
        #
        # # Show plot
        # plt.show()
        reach_node.intersect_in_position_domain(p_lon_max=reach_node.position_rectangle.p_lon_max,
                                                p_lon_min=reach_node.position_rectangle.p_lon_min,
                                                p_lat_max=reach_node.position_rectangle.p_lat_max,
                                                p_lat_min=reach_node.position_rectangle.p_lat_min)
        return [reach_node]


    # def _get_vehicle_intersecting_lanelet_ids(self, step, semantic_model: SemanticModel) -> Optional[Set[int]]:
    #     if vehicle := semantic_model.vehicle_model.find_vehicle_by_id(self.vehicle_id):
    #         return {
    #             intersecting
    #             for lanelet_id in vehicle.lanelet_ids_at_step(step)
    #             for intersecting in
    #             semantic_model.lanelet_model.dict_id_lanelet_to_set_ids_lanelets_intersecting[lanelet_id]
    #         }
    #     else:
    #         return None

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
