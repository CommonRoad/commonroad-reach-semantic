from typing import Optional, Dict

from commonroad_reach.data_structure.reach.reach_interface import ReachableSetInterface

from commonroad_reach_semantic.data_structure.config.semantic_configuration import SemanticConfiguration
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.rule.traffic_rule_interface import TrafficRuleInterface


class SemanticReachableSetInterface(ReachableSetInterface):
    """Interface for reachable set computation with semantic constraints."""

    computation_modes: Dict[int, str] = {
        5: "Semantic labeling reachable set (Python)",
        6: "Semantic labeling reachable set (C++)",
        7: "Semantic on-the-fly reachable set (Python)",
        8: "Semantic on-the-fly reachable set (C++)",
    }

    semantic_model: Optional[SemanticModel]
    rule_interface: Optional[TrafficRuleInterface]

    def __init__(self, config: Optional[SemanticConfiguration] = None, semantic_model: Optional[SemanticModel] = None,
                 rule_interface: Optional[TrafficRuleInterface] = None) -> None:
        """Create a new interface for reachable set computation with semantic constraints.

        :param config: Configuration for the reachable set computation
        :param semantic_model: Semantic model of the environment
        :param rule_interface: Traffic rules to be considered
        """
        self.semantic_model = semantic_model
        self.rule_interface = rule_interface
        super().__init__(config)

    def reset(self, config: SemanticConfiguration, semantic_model: Optional[SemanticModel] = None,
              rule_interface: Optional[TrafficRuleInterface] = None) -> None:
        """Resets configuration of the interface.

        :param config: New configuration
        :param semantic_model: New semantic model
        :param rule_interface: New rule interface
        """
        if semantic_model:
            self.semantic_model = semantic_model
        if rule_interface:
            self.rule_interface = rule_interface

        if config.reachable_set.mode_computation in self.computation_modes:
            self.config = config
            self._reach = None
            self._reachable_set_computed = False
            self._driving_corridor_extractor = None

            if not self.semantic_model or not self.rule_interface:
                raise RuntimeError("Semantic model and rule interface must be specified for semantic reachability.")

            self._instantiate_semantic_reach_set(config.reachable_set.mode_computation)
        else:
            super().reset(config)

    def _instantiate_semantic_reach_set(self, mode_computation: int) -> None:
        """Instantiate the internal reach set object depending on the specified mode.

        :param mode_computation: Mode of computation
        """
        if mode_computation not in self.computation_modes:
            raise ValueError(f"Unknown computation mode: {mode_computation}\nSupported modes:\n" + "\n".join(
                f"{mode}: {description}" for mode, description in self.computation_modes.items()))

        match mode_computation:
            case 5:
                from commonroad_reach_semantic.data_structure.reach.semantic_labeling_reach_set_py import \
                    PySemanticLabelingReachableSet
                self._reach = PySemanticLabelingReachableSet(self.config, self.semantic_model, self.rule_interface)
            case 6:
                from commonroad_reach_semantic.data_structure.reach.semantic_labeling_reach_set_cpp import \
                    CppSemanticLabelingReachableSet
                self._reach = CppSemanticLabelingReachableSet(self.config, self.semantic_model, self.rule_interface)
            case 7:
                from commonroad_reach_semantic.data_structure.reach.semantic_otf_reach_set_py import \
                    PySemanticOTFReachableSet
                self._reach = PySemanticOTFReachableSet(self.config, self.semantic_model, self.rule_interface)
            case 8:
                from commonroad_reach_semantic.data_structure.reach.semantic_otf_reach_set_cpp import \
                    CppSemanticOTFReachableSet
                self._reach = CppSemanticOTFReachableSet(self.config, self.semantic_model, self.rule_interface)
