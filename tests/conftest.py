"""Shared fixtures"""
import os
import pathlib
import sys

import pytest

from commonroad_reach_semantic.data_structure.config.semantic_configuration import SemanticConfiguration
from commonroad_reach_semantic.data_structure.config.semantic_configuration_builder import SemanticConfigurationBuilder
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.reach.semantic_labeling_reach_set_py import PySemanticLabelingReachableSet
from commonroad_reach_semantic.data_structure.reach.semantic_otf_reach_set_py import PySemanticOTFReachableSet
from commonroad_reach_semantic.data_structure.rule import priorities
from commonroad_reach_semantic.data_structure.rule.traffic_rule_interface import TrafficRuleInterface

sys.path.append(os.getcwd())


@pytest.fixture
def config() -> SemanticConfiguration:
    path_root = str(pathlib.Path(__file__).parent.resolve())
    config = SemanticConfigurationBuilder.build_configuration("ZAM_Merge-1_1_T-1", path_root)
    config.update()
    return config


@pytest.fixture
def semantic_model(config: SemanticConfiguration) -> SemanticModel:
    semantic_model = SemanticModel(config)
    semantic_model.determine_traffic_priorities(priorities.dict_traffic_sign_to_priorities)
    return semantic_model


@pytest.fixture
def rule_interface(config: SemanticConfiguration, semantic_model: SemanticModel) -> TrafficRuleInterface:
    return TrafficRuleInterface(config, semantic_model)


@pytest.fixture
def semantic_otf_reachable_set_py(config: SemanticConfiguration, semantic_model: SemanticModel,
                                  rule_interface: TrafficRuleInterface) -> PySemanticLabelingReachableSet:
    return PySemanticLabelingReachableSet(config, semantic_model, rule_interface)


@pytest.fixture
def semantic_otf_reachable_set_py(config: SemanticConfiguration, semantic_model: SemanticModel,
                                  rule_interface: TrafficRuleInterface) -> PySemanticOTFReachableSet:
    return PySemanticOTFReachableSet(config, semantic_model, rule_interface)
