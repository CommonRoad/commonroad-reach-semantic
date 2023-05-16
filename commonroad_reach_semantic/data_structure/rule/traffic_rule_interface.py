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
    semantic_model: SemanticModel
    list_traffic_rules_activated: List[str]
    list_traffic_rules_to_be_concretized: List[str]
    list_specifications_ltl: List[str]
    tpl_checker: TPLChecker
    _cnt_concretized_rules: int

    def __init__(self, config: SemanticConfiguration, semantic_model: SemanticModel) -> None:
        self.config = config
        self.semantic_model = semantic_model
        self.list_traffic_rules_activated = config.traffic_rule.activated_rules
        self.list_traffic_rules_to_be_concretized = list()

        # LTL specifications
        self.list_specifications_ltl = list()

        # TPL specifications
        self.tpl_checker = TPLChecker(config)

        self._cnt_concretized_rules = 0
        for item in self.list_traffic_rules_activated:
            self._parse_traffic_rule(item, allow_abstract_rules=True)

        self.tpl_checker.extract_mandatory_and_forbidden_propositions()

        # if no LTL specification is given, add the trivial one
        if not self.list_specifications_ltl:
            self.list_specifications_ltl.append("true")

        logger.info("Traffic rules concretized.")
        logger.info(f"\t#Rules concretized: {self._cnt_concretized_rules}")

        logger.debug("TrafficRuleInterface instantiated.")

    def _parse_traffic_rule(self, item: str, allow_abstract_rules: bool) -> None:
        """
        Parses the given traffic rules.

        Those rules expressed with Timed Propositional Logic and Linear Temporal Logic will be added to list of
        specifications. The other rules will be concretized to LTL and TPL specifications.

        TPL: examined during propagation of reachable sets
        LTL: examined on the reachability graph with automata-based approach
        """
        tag, specification = item.split()[0], " ".join(item.split()[1:])

        # if the given rule is in the form of a specification
        if tag == "TPL":
            self.tpl_checker.add_specification(specification)
        elif tag == "LTL":
            self.list_specifications_ltl.append(specification)
        else:
            # This is not an LTL or TPL specification
            if allow_abstract_rules:
                # If we are still in the first pass, it could be a traffic rule to concretize
                concrete_specifications = TrafficRule.from_string(item).concretize(self.semantic_model)
                for concrete_specification in concrete_specifications:
                    # Parse the concrete specifications ...
                    # ... but don't allow rules, because they should all be LTL or TPL specifications
                    self._parse_traffic_rule(concrete_specification, allow_abstract_rules=False)
                self._cnt_concretized_rules += 1
            else:
                # Since we expect LTL or TPL specifications and don't allow abstract rules, this is an error
                util_logger.print_and_log_error(f"Rule cannot be parsed: {item}")

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

    def get_combined_ltl_specs(self) -> str:
        return "(" + ") & (".join(self.list_specifications_ltl) + ")"
