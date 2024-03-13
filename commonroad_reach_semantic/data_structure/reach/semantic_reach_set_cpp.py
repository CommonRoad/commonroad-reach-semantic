import logging
from abc import ABC

from commonroad_reach_semantic import pycrreachsem
from commonroad_reach_semantic.data_structure.config.semantic_configuration import SemanticConfiguration
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.reach.semantic_reach_set import SemanticReachableSet
from commonroad_reach_semantic.data_structure.rule.traffic_rule_interface import TrafficRuleInterface

logger = logging.getLogger(__name__)


class CppSemanticReachableSet(SemanticReachableSet, ABC):
    """Abstract base class for C++ implementations of semantic reachable set computation."""

    _reach: pycrreachsem.SemanticReachableSet

    def __init__(self, config: SemanticConfiguration, semantic_model: SemanticModel,
                 rule_interface: TrafficRuleInterface):
        super().__init__(config, semantic_model, rule_interface)

    def _reset_reachable_set_at_step(self, step: int, reachable_set):
        pass

    def compute_drivable_area_at_step(self, step):
        pass

    def compute_reachable_set_at_step(self, step):
        pass

    def compute(self, step_start: int, step_end: int):
        for step in range(step_start, step_end + 1):
            logger.debug(f"Computing reachable set for step {step}")
            self._reach.compute(step, step)
            self._list_steps_computed.append(step)

        if self.config.reachable_set.prune_nodes_not_reaching_final_step:
            self.prune_nodes_not_reaching_final_step()

        self.dict_step_to_drivable_area = self._reach.drivable_area()
        self.dict_step_to_reachable_set = self._reach.reachable_set()
        self.dict_step_to_propagated_set = self._reach.propagated_set()

    def drivable_area_at_step(self, step: int):
        return self._reach.drivable_area_at_step(step)

    def reachable_set_at_step(self, step: int):
        return self._reach.reachable_set_at_step(step)

    def prune_nodes_not_reaching_final_step(self):
        self._reach.prune_nodes_not_reaching_final_step()

    @property
    def labeler(self):
        return self._reach.labeler
