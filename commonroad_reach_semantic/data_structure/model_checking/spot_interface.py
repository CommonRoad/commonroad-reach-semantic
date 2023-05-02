import logging
import time
from typing import List, Optional

import commonroad_reach.utility.logger as util_logger
import spot
from commonroad_reach.data_structure.reach.reach_interface import ReachableSetInterface

from commonroad_reach_semantic.data_structure.model_checking.automaton_graph import AutomatonGraph
from commonroad_reach_semantic.data_structure.model_checking.kripke import KripkeStructure
from commonroad_reach_semantic.data_structure.rule.traffic_rule_interface import TrafficRuleInterface

logger = logging.getLogger(__name__)


class SpotInterface:
    def __init__(self, reach_interface: ReachableSetInterface, rule_interface: TrafficRuleInterface):
        self.reach_interface = reach_interface
        self.rule_interface = rule_interface
        self.config = reach_interface.config
        self.mode_spot = self.config.traffic_rule.mode_spot

        if self.mode_spot not in [1, 2]:
            raise Exception("Specified mode ID is invalid.")

        # formulas-related
        self.list_formulas_ltl = list()
        self.set_propositions_in_ltl_formulas = set()
        self.list_automata_ltl = list()

        # reachability-graph-related
        self.kripke_structure = None
        self.automaton_model = None

        # product-automaton-related
        self.automaton_product = None
        self.graph_automaton: Optional[AutomatonGraph] = None

    def translate_ltl_formulas(self):
        """
        Translates LTL formulas into Büchi automata.
        """
        self.list_formulas_ltl = self.parse_ltl_formulas()
        self.set_propositions_in_ltl_formulas = self.extract_propositions_from_ltl_formulas()

        util_logger.print_and_log_info(logger, f"Translating LTLf to automata...", False)
        time_start = time.time()

        self.list_automata_ltl = self.translate_ltl_formulas_to_automata()

        time_computation = time.time() - time_start
        util_logger.print_and_log_info(logger, f"\tTranslation took: \t{time_computation:.3f}s", False)
        util_logger.print_and_log_info(logger, f"\t#Automata: {len(self.list_automata_ltl)}", False)

    def translate_reachability_graph(self):
        """
        Translates reachability graph into a Kripke structure and then into a Büchi automaton.
        """
        # BDD dict of the specification Büchi automata needs to be shared with the Kripke structure
        KripkeStructure.cls_bdd_dict = self.list_automata_ltl[0].get_dict()
        KripkeStructure.cls_set_propositions_relevant = self.set_propositions_in_ltl_formulas

        # create Kripke structure
        self.kripke_structure = KripkeStructure(self.reach_interface)
        # create transition-based omega automaton from the state-based Kripke structure
        self.automaton_model = self.kripke_structure.construct_twa()

    def check(self):
        """
        Model check the model automaton with the specification automata.

        A product automaton is computed from the model automaton and Büchi automata, which is then converted to a graph
        for later operations.
        """
        util_logger.print_and_log_info(logger, "* Constructing automaton graph...")
        time_start = time.time()

        # compute the product automaton of the model automaton and specification automata
        self.automaton_product = self.compute_product_automaton()

        time_computation = time.time() - time_start
        util_logger.print_and_log_info(logger, f"\tChecking took: \t{time_computation:.3f}s")

        # convert product automaton to a graph
        self.graph_automaton = AutomatonGraph(self.automaton_product, self.kripke_structure)

        util_logger.print_and_log_info(logger, f"\t#Nodes: {len(self.graph_automaton.list_nodes_auto)}")

    def parse_ltl_formulas(self) -> List[str]:
        """
        Parses LTL formulas from the given specifications.
        """
        # concatenate specifications and handle them with a single automaton
        if self.mode_spot == 1:
            formula_ltl_concatenated = ""
            for idx, specification_ltl in enumerate(self.rule_interface.list_specifications_ltl):
                if idx == 0:
                    formula_ltl_concatenated = specification_ltl

                else:
                    formula_ltl_concatenated = f"{formula_ltl_concatenated} & {specification_ltl}"

            return [formula_ltl_concatenated]

        else:
            return self.rule_interface.list_specifications_ltl

    def extract_propositions_from_ltl_formulas(self):
        """
        Returns propositions in the LTL formulas.
        """
        set_propositions = set()

        for formula_ltl in self.list_formulas_ltl:
            set_propositions.update({str(proposition) for proposition in spot.translate(formula_ltl).ap()})

        return set_propositions

    def translate_ltl_formulas_to_automata(self):
        """
        Translates LTL formulas to Büchi automata.

        To build a finite automaton that recognize LTLfs (i.e. LTL with finite semantics), we additionally require the
        "!dead" proposition. The argument "xargs='simul=0'" drastically speed up the translation process.
        """
        list_automata_ltl = [spot.from_ltlf(formula_ltl, "!dead").translate(xargs='simul=0')
                             for formula_ltl in self.list_formulas_ltl]

        return list_automata_ltl

    def compute_product_automaton(self):
        """
        Computes the product automaton of the model automaton and the specification automata.
        """
        if self.mode_spot == 1:
            assert len(self.list_automata_ltl) == 1, "Number of LTL formulas does not equal to 1."
            automaton_product = spot.product(self.automaton_model, self.list_automata_ltl[0])
            automaton_product.purge_dead_states()

            return automaton_product

        else:
            list_automata_product = list()
            for automaton_ltl in self.list_automata_ltl:
                automaton_product = spot.product(self.automaton_model, automaton_ltl)
                automaton_product.purge_dead_states()
                automaton_product.merge_states()
                automaton_product.merge_edges()
                list_automata_product.append(automaton_product)

            if len(list_automata_product) == 1:
                return list_automata_product[0]

            else:
                automaton_product_final = list_automata_product[0]
                for automaton_product in list_automata_product[1:]:
                    automaton_product_final = spot.product(automaton_product_final, automaton_product)
                    automaton_product_final.purge_dead_states()
                    automaton_product_final.merge_states()
                    automaton_product_final.merge_edges()

            return automaton_product_final
