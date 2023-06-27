import networkx as nx
from commonroad_reach.data_structure.reach.reach_interface import ReachableSetInterface


def reachability_graph_to_networkx(reach_interface: ReachableSetInterface) -> nx.Graph:
    """Convert the reachability graph to a networkx graph.

    :param reach_interface: The reachable set interface with the precomputed reachability graph.
    :return: A networkx representation of the reachability graph.
    """
    graph = nx.DiGraph()

    # add nodes
    for step, nodes in reach_interface.reachable_set.items():
        graph.add_nodes_from(nodes, layer=step)
        graph.add_edges_from(
            (node, child) for node in nodes for child in node.list_nodes_child
        )
    return graph


def reachability_graph_nx_layout(graph: nx.Graph, scale: float = None) -> dict:
    """Compute a layout for the reachability graph."""
    return nx.multipartite_layout(graph, subset_key="layer", scale=scale)
