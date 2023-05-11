from typing import List

from commonroad_reach.data_structure.reach.reach_node import ReachNode

import commonroad_reach_semantic.data_structure.reach.predicates.predicate as predicate
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.rule.proposition import Proposition as Prop


class InIntersectionPredicate(predicate.Predicate):

    def to_proposition(self) -> str:
        return Prop.in_intersection()

    def restrict_reach_node_mandatory(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel) -> List[
        ReachNode]:
        lanelet_ids = semantic_model.lanelet_model.set_ids_lanelets_in_intersections
        split_sets = []
        for region in semantic_model.region_model.list_regions:
            if region.set_ids_lanelets.intersection(lanelet_ids):
                if reach_node_new := self._cut_to_region(reach_node, region):
                    split_sets.append(reach_node_new)
        return split_sets

    def restrict_reach_node_forbidden(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel) -> List[
        ReachNode]:
        lanelet_ids = semantic_model.lanelet_model.set_ids_lanelets_in_intersections
        split_sets = []
        for region in semantic_model.region_model.list_regions:
            if region.set_ids_lanelets.isdisjoint(lanelet_ids):
                if reach_node_new := self._cut_to_region(reach_node, region):
                    split_sets.append(reach_node_new)
        return split_sets
