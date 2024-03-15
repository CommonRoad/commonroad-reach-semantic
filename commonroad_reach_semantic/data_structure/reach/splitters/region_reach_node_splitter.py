from typing import List, Tuple

from commonroad_reach.data_structure.reach.reach_node import ReachNode

from commonroad_reach_semantic.data_structure.environment_model.region import Region
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel


class RegionReachNodeSplitter:
    """Splits reach nodes according to regions in the semantic model."""

    semantic_model: SemanticModel

    def __init__(self, semantic_model: SemanticModel) -> None:
        """Create a new splitter for the given semantic model.

        :param semantic_model: The semantic model.
        """
        self.semantic_model = semantic_model

    def split_wrt_regions(self, reachable_set: ReachNode) -> List[Tuple[Region, ReachNode]]:
        """Splits a reachable set with respect to lanelet regions.

        Steps:
            1. Intersect reachable set in the position domain with lanelet regions
            2. Over-approximate and restore to axis-aligned rectangles

        :param reachable_set: The reachable set to split.
        :returns: A list of pairs of regions and the corresponding reachable set.
        """

        list_sets_split = []
        # iterate through regions intersecting with the reachable set
        for region in self.semantic_model.region_model.list_regions:
            # first compute intersection with bounding box
            # --> exact intersection is more expensive, so we only want to compute it if necessary
            # there is no possibility of intersection
            if not region.intersects(reachable_set.position_rectangle.bounds, coordinate_system="CVLN"):
                continue

            # there is a possibility of intersection
            polygon_intersection = region.polygon_cvln.intersection(reachable_set.position_rectangle)

            # empty intersection
            if not polygon_intersection or polygon_intersection.is_empty:
                continue

            restricted_reachable_set = reachable_set.clone()
            # intersect the reachable set with the polygon
            # as we can only represent axis-aligned rectangles, we have to overapproximate by using the polygon bounds
            restricted_reachable_set.intersect_in_position_domain(*polygon_intersection.bounds)

            list_sets_split.append((region, restricted_reachable_set))

        return list_sets_split
