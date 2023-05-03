from abc import ABC

from commonroad_reach.data_structure.reach.reach_set import ReachableSet

from commonroad_reach_semantic.data_structure.config.semantic_configuration import SemanticConfiguration
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.reach.reachable_set_labeler import ReachableSetLabeler
from commonroad_reach_semantic.data_structure.rule.traffic_rule_interface import TrafficRuleInterface


class SemanticReachableSet(ReachableSet, ABC):
    def __init__(self, config: SemanticConfiguration, semantic_model: SemanticModel,
                 rule_interface: TrafficRuleInterface):
        super().__init__(config)
        self.rule_interface = rule_interface
        self.labeler = ReachableSetLabeler(semantic_model)
