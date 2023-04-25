import os

from commonroad_reach.data_structure.configuration_builder import ConfigurationBuilder
from omegaconf import OmegaConf

from commonroad_reach_semantic.data_structure.semantic_configuration import SemanticConfiguration


class SemanticConfigurationBuilder(ConfigurationBuilder):

    @classmethod
    def build_configuration(cls, name_scenario: str, path_root: str = None,
                            dir_config: str = "configurations",
                            dir_config_default: str = "defaults") -> SemanticConfiguration:
        """
        Builds semantic configuration from default, scenario-specific, and commandline config files.

        :param name_scenario: name of the considered scenario
        :param path_root: root path of the package
        :param dir_config: directory storing configurations
        :param dir_config_default: directory storing default configurations
        :return: built configuration
        """
        if path_root is None:
            path_root = os.path.normpath(os.path.join(os.path.dirname(__file__), "../.."))

        if cls.path_root is None:
            cls.set_paths(path_root=path_root, dir_config=dir_config, dir_config_default=dir_config_default)

        config_default = cls.construct_default_configuration()
        config_scenario = cls.construct_scenario_configuration(name_scenario)
        config_cli = OmegaConf.from_cli()
        # configurations coming after overrides the ones coming before
        config_merged = OmegaConf.merge(config_default, config_scenario, config_cli)
        config = SemanticConfiguration(config_merged)

        return config
