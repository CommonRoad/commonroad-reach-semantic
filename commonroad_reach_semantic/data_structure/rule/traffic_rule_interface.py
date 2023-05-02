import logging
from typing import List

import commonroad_reach.utility.logger as util_logger

from commonroad_reach_semantic.data_structure.config.semantic_configuration import SemanticConfiguration
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.rule.tpl_checker import TPLChecker
from commonroad_reach_semantic.data_structure.rule.traffic_rule import TrafficRule

logger = logging.getLogger(__name__)


class TrafficRuleInterface:
    """Class to hold adopted traffic rules"""

    config: SemanticConfiguration
    list_traffic_rules_activated: List[str]
    list_traffic_rules_to_be_concretized: List[str]
    list_specifications_ltl: List[str]
    tpl_checker: TPLChecker

    def __init__(self, config: SemanticConfiguration) -> None:
        self.config = config
        self.list_traffic_rules_activated = config.traffic_rule.activated_rules
        self.list_traffic_rules_to_be_concretized = list()

        # LTL specifications
        self.list_specifications_ltl = list()

        # TPL specifications
        self.tpl_checker = TPLChecker(config)

        for item in self.list_traffic_rules_activated:
            self._parse_traffic_rule(item, add_to_concretize=True)

        logger.info("TrafficRuleInterface instantiated.")

    def _parse_traffic_rule(self, item: str, add_to_concretize: bool = False) -> None:
        """
        Parses the given traffic rules.

        Those rules expressed with Timed Propositional Logic and Linear Temporal Logic will be added to list of
        specifications. The other rules will be concretized afterward.

        TPL: examined during propagation of reachable sets
        LTL: examined on the reachability graph with automata-based approach
        """
        tag, specification = item.split()[0], " ".join(item.split()[1:])

        # if the given rule is in the form of a specification
        if tag == "TPL":
            self.tpl_checker.add_specification(specification)
        elif tag == "LTL":
            self.list_specifications_ltl.append(specification)
        elif add_to_concretize:
            self.list_traffic_rules_to_be_concretized.append(item)
        else:
            util_logger.print_and_log_error(f"Rule cannot be parsed: {item}")

    def concretize_traffic_rules(self, semantic_model: SemanticModel) -> None:
        """
        Concretizes traffic rules with respect to the given semantic model.
        """
        for traffic_rule in self.list_traffic_rules_to_be_concretized:
            list_specifications = TrafficRule.from_string(traffic_rule).concretize(semantic_model)
            for item in list_specifications:
                self._parse_traffic_rule(item)

        # if no LTL specification is given, add the trivial one
        if not self.list_specifications_ltl:
            self.list_specifications_ltl.append("true")

        self.tpl_checker.extract_mandatory_and_forbidden_propositions()

        logger.info("Traffic rules concretized.")
        logger.info(f"\t#Rules concretized: {len(self.list_traffic_rules_to_be_concretized)}")

    def print_summary(self) -> None:
        string = "# ===== Specification Summary ===== #\n"
        string += f"# TPL:\n"
        for specification in self.tpl_checker.list_specifications_tpl:
            string += f"# \t{specification}\n"

        string += f"# LTL:\n"
        for specification in self.list_specifications_ltl:
            string += f"# \t{specification}\n"

        string += "# ================================= #"

        for line in string.split("\n"):
            util_logger.print_and_log_info(logger, line)
