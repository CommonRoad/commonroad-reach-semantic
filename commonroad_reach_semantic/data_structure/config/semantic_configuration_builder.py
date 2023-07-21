import os

from commonroad_reach.data_structure.configuration_builder import ConfigurationBuilder
from omegaconf import OmegaConf

from commonroad_reach_semantic.data_structure.config.semantic_configuration import SemanticConfiguration


class SemanticConfigurationBuilder(ConfigurationBuilder):

    def build_configuration(self, name_scenario: str) -> SemanticConfiguration:
        """
        Builds semantic configuration from default, scenario-specific, and commandline config files.

        :param name_scenario: name of the considered scenario
        :return: built configuration
        """
        config_scenario = self.construct_scenario_configuration(name_scenario)
        config_cli = OmegaConf.from_cli()
        # configurations coming after overrides the ones coming before
        config_merged = OmegaConf.merge(self.config_default, config_scenario, config_cli)
        config = SemanticConfiguration(config_merged)

        return config
