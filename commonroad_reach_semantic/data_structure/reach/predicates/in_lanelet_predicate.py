from typing import List

from commonroad_reach.data_structure.reach.reach_node import ReachNode

import commonroad_reach_semantic.data_structure.reach.predicates.predicate as predicate
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.rule.proposition import Proposition as Prop


class InLaneletPredicate(predicate.Predicate):
    def __init__(self, lanelet_id: int):
        self.lanelet_id = lanelet_id

    def to_proposition(self) -> str:
        return Prop.in_lanelet(self.lanelet_id)

    def restrict_reach_node_mandatory(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel) -> List[
        ReachNode]:
        split_sets = []
        for region in semantic_model.region_model.list_regions:
            if self.lanelet_id in region.set_ids_lanelets:
                if reach_node_new := self._cut_to_region(reach_node, region):
                    split_sets.append(reach_node_new)
        return split_sets

    def restrict_reach_node_forbidden(self, step: int, reach_node: ReachNode, semantic_model: SemanticModel) -> List[
        ReachNode]:
        split_sets = []
        for region in semantic_model.region_model.list_regions:
            if self.lanelet_id not in region.set_ids_lanelets:
                if reach_node_new := self._cut_to_region(reach_node, region):
                    split_sets.append(reach_node_new)
        return split_sets
