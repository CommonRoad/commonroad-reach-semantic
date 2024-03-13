import logging

from commonroad_reach_semantic import pycrreachsem
from commonroad_reach_semantic.data_structure.config.semantic_configuration import SemanticConfiguration
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.reach.semantic_reach_set_cpp import CppSemanticReachableSet
from commonroad_reach_semantic.data_structure.rule.traffic_rule_interface import TrafficRuleInterface

logger = logging.getLogger(__name__)


class CppSemanticOTFReachableSet(CppSemanticReachableSet):
    """Reachable set computation considering temporal constraints on-the-fly with C++ backend."""

    def __init__(self, config: SemanticConfiguration, semantic_model: SemanticModel,
                 rule_interface: TrafficRuleInterface):
        super().__init__(config, semantic_model, rule_interface)

        self._reach = pycrreachsem.SemanticOTFReachableSet(self.config.convert_to_cpp_configuration(),
                                                         self.collision_checker.cpp_collision_checker,
                                                           pycrreachsem.SemanticModel(semantic_model),
                                                           pycrreachsem.TrafficRuleInterface(rule_interface))

        logger.info("CppSemanticOTFReachableSet initialized.")

    @property
    def reachable_set_to_label(self):
        return {reach_set: frozenset(states) for reach_set, states in self._reach.reachable_set_to_label.items()}
