from typing import Dict, Set, Iterator

import networkx as nx
import spot


def create_proposition_formula(set_propositions,
                               dict_proposition_to_bdd_proposition: Dict,
                               list_propositions_in_rules: Set[str], dead: bool = False):
    """
    Creates a proposition formula that is recognized by Spot.

    The proposition "!dead" is attached to all states that are of the Kripke structure. "dead" is only attached to an
    extra state at the end required for interpreting LTL over finite traces.
    """
    formula = None
    # set_propositions_kripke = proposition_holder.propositions() if proposition_holder else set()

    # formula should contain positive or negative statements of propositions in rules
    if list_propositions_in_rules:
        for idx, proposition in enumerate(list_propositions_in_rules):
            if proposition in set_propositions:
                if idx == 0:
                    formula = dict_proposition_to_bdd_proposition[proposition]

                else:
                    formula &= dict_proposition_to_bdd_proposition[proposition]

            else:
                if idx == 0:
                    formula = -dict_proposition_to_bdd_proposition[proposition]

                else:
                    formula &= -dict_proposition_to_bdd_proposition[proposition]

    else:
        # if there is no relevant propositions, the formula turns to "true"
        formula = dict_proposition_to_bdd_proposition[0]

    if not dead:
        formula &= -dict_proposition_to_bdd_proposition["dead"]

    else:
        formula &= dict_proposition_to_bdd_proposition["dead"]

    return formula


def compute_num_paths_in_graph(graph_automaton):
    """
    Returns the number of possible paths in the graph.
    """

    step_end = graph_automaton.kripke_structure.step_end
    for step in range(step_end + 1):
        set_nodes_automaton = graph_automaton.query_automaton_nodes_by_step(step)
        if step == 0:
            for node in set_nodes_automaton:
                node.counter = 1

        else:
            for node in set_nodes_automaton:
                node.counter = sum([node_parent.counter for node_parent in node.set_nodes_auto_parent])

    num_paths = sum([node.counter for node in graph_automaton.query_automaton_nodes_by_step(step_end)])

    return num_paths


def convert_automaton_graph_to_networkx_graph(graph_automaton):
    """
    Converts an automaton graph into a networkx graph.
    """
    G = nx.DiGraph()

    # add nodes
    set_idx_nodes_auto = {node_auto.idx_state_product for node_auto in graph_automaton.list_nodes_auto}
    G.add_nodes_from(set_idx_nodes_auto)

    # add edges
    list_edges_graph = list()
    for node_auto in graph_automaton.list_nodes_auto:
        list_edges_node = [(node_auto.idx_state_product, node_auto_child.idx_state_product)
                           for node_auto_child in node_auto.set_nodes_auto_child]
        list_edges_graph += list_edges_node
    G.add_edges_from(list_edges_graph)

    return G


def conjuncts(formula: spot.formula) -> Iterator[spot.formula]:
    """Iterate over all conjuncts of a spot formula.

    list(conjuncts(a & b)) == [a, b]
    list(conjuncts(a | b)) == [a | b]
    """
    if formula._is(spot.op_And):
        for child in formula:
            yield child
    else:
        yield formula


def disjuncts(formula: spot.formula) -> Iterator[spot.formula]:
    """Iterate over all disjuncts of a spot formula.

    list(disjuncts(a | b)) == [a, b]
    list(disjuncts(a & b)) == [a & b]
    """
    if formula._is(spot.op_Or):
        for child in formula:
            yield child
    else:
        yield formula


def extract_atomic_proposition(literal: spot.formula) -> tuple[str, bool]:
    """Extract the atomic proposition from a (negated) literal.
    
    :param literal: Formula that is either a literal or a negated literal
    :return: Name of the atomic proposition and whether it is negated or not
    """
    if literal._is(spot.op_Not) and literal[0]._is(spot.op_ap):
        return literal[0].ap_name(), True
    elif literal._is(spot.op_ap):
        return literal.ap_name(), False
    else:
        raise ValueError(f"{literal} is not a (negated) literal")
