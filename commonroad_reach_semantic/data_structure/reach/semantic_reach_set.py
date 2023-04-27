from abc import ABC

from commonroad_reach.data_structure.reach.reach_set import ReachableSet

from commonroad_reach_semantic.data_structure.semantic_configuration import SemanticConfiguration
from commonroad_reach_semantic.data_structure.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.traffic_rule_interface import TrafficRuleInterface


class SemanticReachableSet(ReachableSet, ABC):
    def __init__(self, config: SemanticConfiguration, semantic_model: SemanticModel,
                 rule_interface: TrafficRuleInterface):
        super().__init__(config)
        self.semantic_model = semantic_model
        self.rule_interface = rule_interface
