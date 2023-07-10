from typing import Optional

from commonroad_reach.data_structure.reach.reach_interface import ReachableSetInterface
from commonroad_reach.data_structure.reach.reach_set import ReachableSet

from commonroad_reach_semantic.data_structure.config.semantic_configuration import SemanticConfiguration
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.rule.traffic_rule_interface import TrafficRuleInterface


class SemanticReachableSetInterface(ReachableSetInterface):
    """Interface for reachable set computation with semantic constraints."""

    def __init__(self, config: Optional[SemanticConfiguration] = None, semantic_model: Optional[SemanticModel] = None,
                 rule_interface: Optional[TrafficRuleInterface] = None) -> None:
        # self.semantic_model = semantic_model
        # self.rule_interface = rule_interface
        super().__init__(config)

    def reset(self, config: SemanticConfiguration) -> None:
        """Resets configuration of the interface."""
        # if config.reachable_set.mode_computation in [5, 6, 7, 8]:
        #     self.config = config
        #     self._reach = None
        #     self._reachable_set_computed = False
        #     self._driving_corridor_extractor = None
        #     match config.reachable_set.mode_computation:
        #         case 5:
        #             self._reach = PySemanticLabelingReachableSet(config, self.semantic_model, self.rule_interface)
        #         case 6:
        #             self._reach = CppSemanticLabelingReachableSet(config, self.semantic_model, self.rule_interface)
        #         case 7:
        #             self._reach = PySemanticSplittingOTFReachableSet(config, self.semantic_model, self.rule_interface)
        #         case 8:
        #             self._reach = CppSemanticSplittingOTFReachableSet(config, self.semantic_model, self.rule_interface)
        # else:
        #     super().reset(config)
        self.config = config
        self._reach = None
        self._reachable_set_computed = False
        self._driving_corridor_extractor = None

    def set_reach(self, reach: ReachableSet):
        """Sets the reach object."""
        self._reach = reach
