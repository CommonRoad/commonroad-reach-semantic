import logging
from typing import Set, Dict

import commonroad_reach.utility.logger as util_logger
import numpy as np
import spot
from commonroad.scenario.traffic_sign import TrafficSignIDZamunda

from commonroad_reach_semantic.data_structure.config.semantic_configuration import SemanticConfiguration
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.rule.tpl_checker import TPLChecker
from commonroad_reach_semantic.data_structure.rule.traffic_rule import NoBackwardDrivingRule, NoOppositeDrivingRule, \
    LineMarkingRule, \
    TrafficLightRule, PriorityRule, RightBeforeLeftRule, LeftTurningRule, TrafficRule

logger = logging.getLogger(__name__)


class TrafficRuleInterface:
    """Class to hold adopted traffic rules"""

    set_identifiers_tpl: Set[str] = {"(", ")", "->", "<->", "&", "|", "!", "xor"}

    dict_traffic_rule_to_object: Dict[str, TrafficRule] = {
        "NoBackwardDrivingRule": NoBackwardDrivingRule(),
        "NoOppositeDrivingRule": NoOppositeDrivingRule(),
        "LineMarkingRule": LineMarkingRule(),
        "TrafficLightRule": TrafficLightRule(),
        "PriorityRule": PriorityRule(),
        "RightBeforeLeftRule": RightBeforeLeftRule(),
        "LeftTurningRule": LeftTurningRule(),
    }
    dict_traffic_sign_to_priorities: Dict[TrafficSignIDZamunda, Dict[str, float]] = {
        TrafficSignIDZamunda.ADDITION_LEFT_TURNING_PRIORITY_WITH_OPPOSITE_RIGHT_YIELD: {"left": 5, "straight": 4,
                                                                                        "right": 4, "index": 1},
        TrafficSignIDZamunda.ADDITION_LEFT_TURNING_PRIORITY_WITH_OPPOSITE_YIELD: {"left": 5, "straight": 4,
                                                                                  "right": -np.inf, "index": 2},
        TrafficSignIDZamunda.ADDITION_LEFT_TURNING_PRIORITY_WITH_RIGHT_YIELD: {"left": 5, "straight": -np.inf,
                                                                               "right": 4, "index": 3},
        TrafficSignIDZamunda.ADDITION_RIGHT_TURNING_PRIORITY_WITH_OPPOSITE_LEFT_YIELD: {"left": 4, "straight": 4,
                                                                                        "right": 5, "index": 4},
        TrafficSignIDZamunda.ADDITION_RIGHT_TURNING_PRIORITY_WITH_OPPOSITE_YIELD: {"left": -np.inf, "straight": 4,
                                                                                   "right": 5, "index": 5},
        TrafficSignIDZamunda.ADDITION_RIGHT_TURNING_PRIORITY_WITH_LEFT_YIELD: {"left": 4, "straight": -np.inf,
                                                                               "right": 5, "index": 6},
        TrafficSignIDZamunda.ADDITION_LEFT_TRAFFIC_PRIORITY_WITH_STRAIGHT_RIGHT_YIELD: {"left": 2, "straight": 2,
                                                                                        "right": 2, "index": 7},
        TrafficSignIDZamunda.ADDITION_LEFT_TRAFFIC_PRIORITY_WITH_STRAIGHT_YIELD: {"left": 2, "straight": 2,
                                                                                  "right": -np.inf, "index": 8},
        TrafficSignIDZamunda.ADDITION_RIGHT_TRAFFIC_PRIORITY_WITH_STRAIGHT_LEFT_YIELD: {"left": 2, "straight": 2,
                                                                                        "right": 2, "index": 9},
        TrafficSignIDZamunda.ADDITION_RIGHT_TRAFFIC_PRIORITY_WITH_STRAIGHT_YIELD: {"left": -np.inf, "straight": 2,
                                                                                   "right": 2, "index": 10},
        TrafficSignIDZamunda.PRIORITY: {"left": 4, "straight": 5, "right": 4, "index": 11},
        TrafficSignIDZamunda.RIGHT_OF_WAY: {"left": 4, "straight": 5, "right": 4, "index": 12},
        TrafficSignIDZamunda.YIELD: {"left": 2, "straight": 2, "right": 2, "index": 13},
        TrafficSignIDZamunda.STOP: {"left": 1, "straight": 1, "right": 1, "index": 14},
        TrafficSignIDZamunda.WARNING_RIGHT_BEFORE_LEFT: {"left": 3, "straight": 3, "right": 3, "index": 15},
        TrafficSignIDZamunda.GREEN_ARROW: {"left": -np.inf, "straight": -np.inf, "right": 0, "index": 16},
    }

    def __init__(self, config: SemanticConfiguration) -> None:
        self.config = config
        self.list_traffic_rules_activated = config.traffic_rule.activated_rules
        self.list_traffic_rules_to_be_concretized = list()
        self.list_specifications_tpl = list()
        self.list_specifications_ltl = list()
        self.set_clauses_tpl = set()
        self.set_clauses_ltl = set()

        self.dict_step_to_forbidden_transitions_line_marking = \
            {step: set() for step in range(self.config.planning.steps_computation + 1)}
        self.dict_step_to_forbidden_transitions_traffic_light = \
            {step: set() for step in range(self.config.planning.steps_computation + 1)}

        for item in self.list_traffic_rules_activated:
            self._parse_traffic_rule(item, add_to_concretize=True)

        self.tpl_checker = TPLChecker(config, self.list_specifications_tpl)

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
            self.list_specifications_tpl.append(specification)
            self.set_clauses_tpl.update(self.extract_clauses(self.set_identifiers_tpl, specification))

        elif tag == "LTL":
            self.list_specifications_ltl.append(specification)
            self.set_clauses_ltl.update({str(proposition) for proposition in spot.translate(specification).ap()})

        else:
            if add_to_concretize:
                self.list_traffic_rules_to_be_concretized.append(item)

            else:
                print(f"<TrafficRuleInterface> Rule cannot be parsed: {item}")

    @staticmethod
    def extract_clauses(set_identifiers: Set[str], specification: str) -> Set[str]:
        """
        Extracts clauses from the given specification by removing the identifiers.
        """
        # remove brackets indicating time intervals
        while specification.find("[") >= 0 and specification.find("]") >= 0:
            index_bracket_left = specification.find("[")
            index_bracket_right = specification.find("]")
            specification = specification[0:index_bracket_left] + specification[index_bracket_right + 1:]

        # delete identifiers
        for identifier in set_identifiers:
            specification = specification.replace(identifier, "")

        return set(specification.split())

    def concretize_traffic_rules(self, semantic_model: SemanticModel) -> None:
        """
        Concretizes traffic rules with respect to the given semantic model.
        """
        for traffic_rule in self.list_traffic_rules_to_be_concretized:
            list_specifications = self.dict_traffic_rule_to_object[traffic_rule].concretize(semantic_model)
            if not list_specifications:
                continue

            for item in list_specifications:
                self._parse_traffic_rule(item)

        self._append_default_specifications()
        self.tpl_checker.extract_mandatory_and_forbidden_propositions()

        logger.info("Traffic rules concretized.")
        logger.info(f"\t#Rules concretized: {len(self.list_traffic_rules_to_be_concretized)}")

    def _append_default_specifications(self) -> None:
        """Appends default specifications if the lists are empty."""
        if not self.list_specifications_ltl:
            self.list_specifications_ltl.append(f"true")

    def print_summary(self) -> None:
        string = "# ===== Specification Summary ===== #\n"
        string += f"# TPL:\n"
        for specification in self.list_specifications_tpl:
            string += f"# \t{specification}\n"

        string += f"# LTL:\n"
        for specification in self.list_specifications_ltl:
            string += f"# \t{specification}\n"

        string += "# ================================= #"

        for line in string.split("\n"):
            util_logger.print_and_log_info(logger, line)
