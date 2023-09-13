import copy
import logging
import os
from pathlib import Path
from typing import List, Tuple, Union, Set, Dict, FrozenSet, Iterable

import commonroad_reach.utility.logger as util_logger
import commonroad_reach.utility.visualization as reach_visualization
import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
import seaborn as sns
# from commonroad_reach_semantic import pycrreachs as reach
from commonroad.geometry.shape import Polygon
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.visualization.draw_params import MPDrawParams
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad_reach.data_structure.reach.reach_interface import ReachableSetInterface
from commonroad_reach.data_structure.reach.reach_node import ReachNode
from commonroad_reach.utility import coordinate_system as util_coordinate_system
from pyvis.network import Network

import commonroad_reach_semantic.utility.graph as util_graph
from commonroad_reach_semantic.data_structure.config.semantic_configuration import SemanticConfiguration
from commonroad_reach_semantic.data_structure.driving_corridor_extractor import DrivingCorridor
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.model_checking.kripke_node import KripkeNode
from commonroad_reach_semantic.data_structure.model_checking.spot_interface import SpotInterface
from commonroad_reach_semantic.data_structure.rule.proposition_holder import PropositionHolder

logger = logging.getLogger(__name__)
logging.getLogger('PIL').setLevel(logging.WARNING)
logging.getLogger('matplotlib.font_manager').setLevel(logging.WARNING)


class ColorMapper:
    def __init__(self, reach_interface: ReachableSetInterface, steps: List[int]):
        # determine number of colors

        # different_propositions = {
        #     frozenset(reach_interface._reach.labeler.reachable_set_to_propositions[reach_node].set_propositions)
        #     for step in steps
        #     for reach_node in reach_interface.reachable_set_at_step(step)
        # }
        #
        # self.num_colors = len(different_propositions)
        # self.palette = sns.color_palette("rainbow", self.num_colors)

        self.id_color_max = 0
        self.dict_hash_to_id_color: Dict[int, int] = dict()

    def map_to_color(self, set_propositions: Set[str]):
        hash_value = hash(frozenset(set_propositions))

        try:
            id_color = self.dict_hash_to_id_color[hash_value]

        except KeyError:
            id_color = self.id_color_max
            self.dict_hash_to_id_color[hash_value] = id_color

            self.id_color_max += 1

        return self.palette[id_color]


def groups_from_states(node_to_states: Dict[ReachNode, FrozenSet[int]]) -> Dict[ReachNode, int]:
    """Create groups from automaton state labels."""
    states_to_group = {states: i for i, states in enumerate(set(node_to_states.values()))}
    return {node: states_to_group[states] for node, states in node_to_states.items()}


def groups_from_propositions(node_to_propositions: Dict[ReachNode, PropositionHolder]) -> Dict[ReachNode, int]:
    """Create groups from proposition labels."""
    propositions_to_group = {
        propositions: i
        for i, propositions in enumerate({
            frozenset(prop_holder.set_propositions) for prop_holder in node_to_propositions.values()
        })
    }
    return {
        node: propositions_to_group[frozenset(prop_holder.set_propositions)]
        for node, prop_holder in node_to_propositions.items()
    }


def plot_reach_graph(reach_interface: ReachableSetInterface, figsize: Tuple = None, path_output: str = None, node_to_group: Dict[ReachNode, int] = None):
    """Plot the reachability graph."""
    config: SemanticConfiguration = reach_interface.config
    path_output = path_output or config.general.path_output
    Path(path_output).mkdir(parents=True, exist_ok=True)

    figsize = figsize if figsize else (25, 15)

    if config.debug.save_plots:
        # clear previous plot
        plt.cla()
    else:
        # create new figure
        plt.figure(figsize=figsize)

    g = util_graph.reachability_graph_to_networkx(reach_interface)
    colors = [node_to_group[node] for node in g.nodes] if node_to_group is not None else "#1f78b4"
    nx.draw_networkx(g, pos=util_graph.reachability_graph_nx_layout(g), with_labels=False, node_size=50, width=0.5, node_color=colors, cmap="tab10")

    if config.debug.save_plots:
        reach_visualization.save_fig(False, path_output, 0, identifier="graph", verbose=True)
    else:
        plt.show()

    # clear our plot
    plt.clf()


def show_interactive_reach_graph(reach_interface: ReachableSetInterface, *,
                                 use_images: bool = True, path_output: str = None, file_name: str = "reach_graph",
                                 width: str = "100%", height: str = "1000px", scale: float = 5000, draggable: bool = True,
                                 node_to_group: Dict[ReachNode, int] = None) -> None:
    """Show the reachability graph in an interactive plot.

    :param reach_interface: ReachableSetInterface storing the reachability graph.
    :param use_images: Plot scenario for each node instead of just using a circle.
    :param path_output: Path to output directory.
    :param file_name: Name of the graph HTML file.
    :param width: Width of the plot.
    :param height: Height of the plot.
    :param scale: Scale factor for distances between nodes.
    :param draggable: Whether the nodes can be dragged around.
    :param node_to_group: Mapping from reach nodes to groups for coloring nodes and edges.
    """

    config: SemanticConfiguration = reach_interface.config
    path_output = path_output or config.general.path_output
    Path(path_output).mkdir(parents=True, exist_ok=True)

    g = util_graph.reachability_graph_to_networkx(reach_interface)
    pos = util_graph.reachability_graph_nx_layout(g, scale=scale)
    n = Network(height, width, directed=True)

    # not using n.from_nx(g), because it requires the nodes to be strings or ints
    # for positioning of nodes: https://stackoverflow.com/questions/74108243/pyvis-is-there-a-way-to-disable-physics-without-losing-graphs-layout
    util_logger.print_and_log_info(logger, "* Plotting individual reach nodes...")
    for i, node in enumerate(g.nodes()):
        group = node_to_group[node] if node_to_group is not None else None
        title = f"Lon: [{node.p_lon_min:.4f}; {node.p_lon_max:.4f}] Lat: [{node.p_lat_min:.4f}; {node.p_lat_max:.4f}]"
        shared_options = {"x": pos[node][0], "y": -pos[node][1], "size": 100, "physics": False, "title": title, "group": group}
        if use_images:
            image_path = _plot_reach_node_for_interactive(reach_interface, node, path_output)
            n.add_node(node.id, shape="image", image=os.path.join(os.path.curdir, image_path), **shared_options)
            if i % 5 == 0:
                util_logger.print_and_log_info(logger, f"\tSaving {os.path.join(path_output, image_path)}")
        else:
            n.add_node(node.id, **shared_options)

    for src, dst in g.edges():
        n.add_edge(src.id, dst.id, width=2, physics=False)

    n.toggle_physics(False)
    n.toggle_drag_nodes(draggable)

    # need to change workdir so that pyvis puts its libraries in the right place
    prevdir = os.getcwd()
    os.chdir(path_output)
    n.show(f"{file_name}.html")
    os.chdir(prevdir)


def _plot_reach_node_for_interactive(reach_interface: ReachableSetInterface, reach_node: ReachNode, output_path: str) -> str:
    """Plot the scenario with a single reach node.

    Intended for use with interactive plotting.

    :return: Path to the image relative to output_path.
    """
    config: SemanticConfiguration = reach_interface.config

    relative_figure_path = "img"
    absolute_figure_path = os.path.join(output_path, relative_figure_path)
    Path(absolute_figure_path).mkdir(parents=True, exist_ok=True)

    figsize = (25, 15)
    plot_limits = reach_visualization.compute_plot_limits_from_reachable_sets(reach_interface)
    draw_params = _create_draw_params(config)

    # clear previous plot
    plt.cla()

    renderer = MPRenderer(plot_limits=plot_limits, figsize=figsize)

    draw_params.time_begin = reach_node.step * round(config.planning.dt / config.scenario.dt)
    draw_reachable_sets([reach_node], config, renderer, draw_params)
    _draw_scenario_elements(config, renderer, draw_params)

    # settings and adjustments
    plt.rc("axes", axisbelow=True)
    ax = plt.gca()
    ax.set_aspect("equal")
    plt.margins(0, 0)
    renderer.render()

    filename = f"reach_node_{reach_node.id:010d}.png"
    plt.savefig(os.path.join(absolute_figure_path, filename), format="png", bbox_inches="tight", transparent=False)
    return os.path.join(relative_figure_path, filename)


def plot_scenario_with_reachable_sets(reach_interface: ReachableSetInterface, figsize: Tuple = (25, 15),
                                      step_start: int = 0, step_end: int = 0, steps: List[int] = None,
                                      plot_limits: List = None, path_output: str = None,
                                      save_gif: bool = True, duration: float = None):
    """
    Plots scenario with computed reachable sets.
    """
    config: SemanticConfiguration = reach_interface.config
    scenario = config.scenario

    path_output = path_output or config.general.path_output
    Path(path_output).mkdir(parents=True, exist_ok=True)

    plot_limits = plot_limits or reach_visualization.compute_plot_limits_from_reachable_sets(reach_interface)
    draw_params = _create_draw_params(config)

    step_start = step_start or reach_interface.step_start
    step_end = step_end or reach_interface.step_end
    if steps:
        steps = [step for step in steps if step <= step_end + 1]
    else:
        # add additional step_start for dirty fixing an issue in plotting the traffic signs
        steps = [step_start] + list(range(step_start, step_end + 1))
    duration = duration if duration else config.planning.dt

    # mapper = ColorMapper(reach_interface, steps)

    util_logger.print_and_log_info(logger, "* Plotting reachable sets...")
    renderer = MPRenderer(plot_limits=plot_limits, figsize=figsize) if config.debug.save_plots else None
    for step in steps:
        if config.debug.save_plots:
            # clear previous plot
            plt.cla()
        else:
            # create new figure
            plt.figure(figsize=figsize)
            renderer = MPRenderer(plot_limits=plot_limits)

        time_step = step * round(config.planning.dt / config.scenario.dt)
        draw_params.time_begin = time_step

        list_nodes = reach_interface.reachable_set_at_step(step)
        draw_reachable_sets(list_nodes, config, renderer, draw_params)
        _draw_scenario_elements(config, renderer, draw_params)

        # settings and adjustments
        plt.rc("axes", axisbelow=True)
        ax = plt.gca()
        ax.set_aspect("equal")
        ax.set_title(f"$t = {time_step / 10.0:.1f}$ [s]", fontsize=28)
        ax.set_xlabel(f"$s$ [m]", fontsize=28)
        ax.set_ylabel("$d$ [m]", fontsize=28)
        plt.margins(0, 0)
        renderer.render()

        if config.debug.save_plots:
            reach_visualization.save_fig(save_gif, path_output, step, verbose=(step % 5 == 0))
        else:
            plt.show()

    if config.debug.save_plots and save_gif:
        reach_visualization.make_gif(path_output, "png_reach_", steps, str(scenario.scenario_id), duration)

    util_logger.print_and_log_info(logger, "\tReachable sets plotted.")

    if config.debug.save_config:
        config.save(path_output, str(scenario.scenario_id))
        util_logger.print_and_log_debug(logger, "\tConfiguration file saved.", verbose=False)


def plot_scenario_with_regions(semantic_model: SemanticModel, coordinate_system: str = "CVLN",
                               figsize: Tuple = (25, 15), plot_limits: Union[List] = None, path_output: str = None):
    """
    Plots scenario with computed lanelet regions.
    """
    config: SemanticConfiguration = semantic_model.config
    scenario = config.scenario

    path_output = path_output or config.general.path_output
    Path(path_output).mkdir(parents=True, exist_ok=True)

    plot_limits = plot_limits or compute_plot_limits_from_lanelet_network(semantic_model.lanelet_model.local_lanelet_network)
    draw_params = reach_visualization.generate_default_drawing_parameters(config)

    util_logger.print_and_log_info(logger, "* Plotting lanelet regions...")
    renderer = MPRenderer(plot_limits=plot_limits, figsize=figsize) if config.debug.save_plots else None

    draw_params.time_begin = 0
    scenario.lanelet_network.draw(renderer, draw_params)
    draw_regions(semantic_model, coordinate_system, renderer)

    # plot traffic signs
    for sign in scenario.lanelet_network.traffic_signs:
        sign.draw(renderer)

    plt.rc("axes", axisbelow=True)
    ax = plt.gca()
    ax.set_aspect("equal")
    ax.set_xlabel(f"$s$ [m]", fontsize=28)
    ax.set_ylabel("$d$ [m]", fontsize=28)
    plt.margins(0, 0)
    renderer.render()

    if config.debug.save_plots:
        reach_visualization.save_fig(False, path_output, 0, "region")
    else:
        plt.show()

    util_logger.print_and_log_info(logger, "\tLanelet regions plotted.")


def plot_scenario_with_kripke_nodes(spot_interface: SpotInterface, plot_accepting: bool = True,
                                    figsize: Tuple = (25, 15),
                                    step_start: int = 0, step_end: int = 0, steps: List[int] = None,
                                    plot_limits: Union[List] = None, path_output: str = None,
                                    save_gif: bool = True, duration: float = None):
    """
    Plots scenario with accepting kripke nodes.
    """
    config: SemanticConfiguration = spot_interface.reach_interface.config
    reach_interface = spot_interface.reach_interface
    scenario = config.scenario

    path_output = path_output or config.general.path_output
    Path(path_output).mkdir(parents=True, exist_ok=True)

    plot_limits = plot_limits or reach_visualization.compute_plot_limits_from_reachable_sets(reach_interface)
    draw_params = _create_draw_params(config)

    step_start = step_start or reach_interface.step_start
    step_end = step_end or reach_interface.step_end
    if steps:
        steps = [step for step in steps if step <= step_end + 1]
    else:
        # add additional step_start for dirty fixing an issue in plotting the traffic signs
        steps = [step_start] + list(range(step_start, step_end + 1))
    duration = duration if duration else config.planning.dt

    if plot_accepting:
        dict_step_to_set_nodes_kripke = spot_interface.graph_automaton.retrieve_accepting_kripke_nodes()

    else:
        dict_step_to_set_nodes_kripke = spot_interface.kripke_structure.kripke_nodes(merge=True)

    # mapper = ColorMapper(reach_interface, steps)

    util_logger.print_and_log_info(logger, "* Plotting reachable sets in accepting kripke nodes...")
    renderer = MPRenderer(plot_limits=plot_limits, figsize=figsize) if config.debug.save_plots else None
    for step in steps:
        if config.debug.save_plots:
            # clear previous plot
            plt.cla()
        else:
            # create new figure
            plt.figure(figsize=figsize)
            renderer = MPRenderer(plot_limits=plot_limits)

        time_step = step * round(config.planning.dt / config.scenario.dt)
        draw_params.time_begin = time_step

        draw_kripke_nodes(dict_step_to_set_nodes_kripke[step], config, renderer, draw_params)
        _draw_scenario_elements(config, renderer, draw_params)

        # settings and adjustments
        plt.rc("axes", axisbelow=True)
        ax = plt.gca()
        ax.set_aspect("equal")
        ax.set_title(f"$t = {time_step / 10.0:.1f}$ [s]", fontsize=28)
        ax.set_xlabel(f"$s$ [m]", fontsize=28)
        ax.set_ylabel("$d$ [m]", fontsize=28)
        plt.margins(0, 0)
        renderer.render()

        if config.debug.save_plots:
            reach_visualization.save_fig(save_gif, path_output, step, identifier="kripke", verbose=(step % 5 == 0))
        else:
            plt.show()

    if config.debug.save_plots and save_gif:
        reach_visualization.make_gif(path_output, "png_kripke_", steps, str(scenario.scenario_id), duration)

    util_logger.print_and_log_info(logger, "\tReachable sets of Kripke nodes plotted.")


def plot_scenario_with_driving_corridor(spot_interface: SpotInterface, corridor: DrivingCorridor, figsize: Tuple = (25, 15),
                                        step_start: int = 0, step_end: int = 0, steps: List[int] = None,
                                        plot_limits: Union[List] = None, path_output: str = None,
                                        save_gif: bool = True, duration: float = None):
    """
    Plots scenario with driving corridor.
    """
    if not corridor:
        util_logger.print_and_log_info(logger, "No driving corridor provided.")
        return

    config: SemanticConfiguration = spot_interface.reach_interface.config
    reach_interface = spot_interface.reach_interface

    path_output = path_output or config.general.path_output
    Path(path_output).mkdir(parents=True, exist_ok=True)

    plot_limits = plot_limits or reach_visualization.compute_plot_limits_from_reachable_sets(reach_interface)
    draw_params = _create_draw_params(config)

    step_start = step_start or reach_interface.step_start
    step_end = step_end or reach_interface.step_end
    if steps:
        steps = [step for step in steps if step <= step_end + 1]
    else:
        # add additional step_start for dirty fixing an issue in plotting the traffic signs
        steps = [step_start] + list(range(step_start, step_end + 1))
    duration = duration if duration else config.planning.dt

    dict_step_to_set_nodes_kripke = corridor.retrieve_kripke_nodes()

    # mapper = ColorMapper(reach_interface, steps)

    util_logger.print_and_log_info(logger, "* Plotting driving corridor...")
    renderer = MPRenderer(plot_limits=plot_limits, figsize=figsize) if config.debug.save_plots else None
    for step in steps:
        if config.debug.save_plots:
            # clear previous plot
            plt.cla()
        else:
            # create new figure
            plt.figure(figsize=figsize)
            renderer = MPRenderer(plot_limits=plot_limits)

        time_step = step * round(config.planning.dt / config.scenario.dt)
        draw_params.time_begin = time_step

        set_nodes_reach = set()
        for node_kripke in dict_step_to_set_nodes_kripke[step]:
            set_nodes_reach.update(node_kripke.set_nodes_reach)
        draw_reachable_sets(set_nodes_reach, config, renderer, draw_params)
        _draw_scenario_elements(config, renderer, draw_params)

        # settings and adjustments
        plt.rc("axes", axisbelow=True)
        ax = plt.gca()
        ax.set_aspect("equal")
        ax.set_title(f"$t = {time_step / 10.0:.1f}$ [s]", fontsize=28)
        ax.set_xlabel(f"$s$ [m]", fontsize=28)
        ax.set_ylabel("$d$ [m]", fontsize=28)
        plt.margins(0, 0)
        renderer.render()

        if config.debug.save_plots:
            reach_visualization.save_fig(save_gif, path_output, step, identifier="corridor", verbose=(step % 5 == 0))
        else:
            plt.show()

    if config.debug.save_plots and save_gif:
        reach_visualization.make_gif(path_output, "png_corridor_", steps, str(config.scenario.scenario_id), duration)

    util_logger.print_and_log_info(logger, "\tDriving corridor plotted.")


def compute_plot_limits_from_lanelet_network(lanelet_network: LaneletNetwork, margin: int = 20):
    list_vertices = []
    for lanelet in lanelet_network.lanelets:
        for vertex in lanelet.center_vertices:
            list_vertices.append(vertex)

    x_min = min([x for x, y in list_vertices])
    x_max = max([x for x, y in list_vertices])
    y_min = min([y for x, y in list_vertices])
    y_max = max([y for x, y in list_vertices])

    plot_limits = [x_min - margin, x_max + margin, y_min - margin, y_max + margin]

    return plot_limits


def draw_reachable_sets(nodes: Iterable[ReachNode], config: SemanticConfiguration, renderer: MPRenderer, draw_params: MPDrawParams) -> None:
    coordinate_system = config.planning.coordinate_system

    if coordinate_system == "CART":
        for node in nodes:
            vertices = node.position_rectangle.vertices
            # draw_params.shape.facecolor = mapper.map_to_color(reach_interface._reach.labeler.reachable_set_to_propositions[node].set_propositions)
            Polygon(vertices=np.array(vertices)).draw(renderer, draw_params)

    elif coordinate_system == "CVLN":
        for node in nodes:
            position_rectangle = node.position_rectangle
            list_polygons_cart = util_coordinate_system.convert_to_cartesian_polygons(position_rectangle,
                                                                                      config.planning.CLCS, True)
            # draw_params.shape.facecolor = mapper.map_to_color(reach_interface._reach.labeler.reachable_set_to_propositions[node].set_propositions)
            for polygon in list_polygons_cart:
                Polygon(vertices=np.array(polygon.vertices)).draw(renderer, draw_params)
    else:
        raise RuntimeError(f"Unknown coordinate system {coordinate_system}. Valid values are 'CART' and 'CVLN'.")


def draw_kripke_nodes(set_nodes_kripke: Set[KripkeNode], config: SemanticConfiguration, renderer: MPRenderer, draw_params: MPDrawParams) -> None:
    for node_kripke in set_nodes_kripke:
        draw_params_nodes = copy.deepcopy(draw_params)
        draw_reachable_sets(node_kripke.set_nodes_reach, config, renderer, draw_params_nodes)


def draw_regions(semantic_model: SemanticModel, coordinate_system: str, renderer):
    config: SemanticConfiguration = semantic_model.config
    num_colors = len(semantic_model.region_model.list_regions)
    palette = sns.color_palette("rainbow", num_colors)

    idx_palette = -1
    for region in semantic_model.region_model.list_regions:
        if coordinate_system == "CART":
            polygon_region = region.polygon_cart
            list_vertices_cart = [(x, y) for x, y in zip(polygon_region.shapely_object.exterior.coords.xy[0],
                                                         polygon_region.shapely_object.exterior.coords.xy[1])]

        elif coordinate_system == "CVLN":
            polygon_region = region.polygon_cvln
            list_vertices_cvln = [(s, d) for s, d in zip(polygon_region.shapely_object.exterior.coords.xy[0],
                                                         polygon_region.shapely_object.exterior.coords.xy[1])]
            list_vertices_cart = []
            for (s, d) in list_vertices_cvln:
                try:
                    x, y = config.planning.CLCS.convert_to_cartesian_coords(s, d)
                    list_vertices_cart.append((x, y))

                except ValueError:
                    continue

        else:
            raise RuntimeError(f"Unknown coordinate system {coordinate_system}. Valid values are 'CART' and 'CVLN'.")

        if list_vertices_cart:
            list_vertices_cart.append(list_vertices_cart[0])

            idx_palette += 1
            draw_params = MPDrawParams()
            draw_params.shape.facecolor = palette[idx_palette % num_colors]
            draw_params.shape.edgecolor = palette[idx_palette % num_colors]
            draw_params.shape.zorder = 10

            Polygon(vertices=np.array(list_vertices_cart)).draw(renderer, draw_params)


def _create_draw_params(config: SemanticConfiguration) -> MPDrawParams:
    palette = sns.color_palette("GnBu_d", 3)
    edge_color = (palette[0][0] * 0.75, palette[0][1] * 0.75, palette[0][2] * 0.75)
    # generate default drawing parameters
    draw_params = reach_visualization.generate_default_drawing_parameters(config)
    draw_params.shape.facecolor = palette[0]
    draw_params.shape.edgecolor = edge_color
    return draw_params


def _draw_scenario_elements(config: SemanticConfiguration, renderer: MPRenderer, draw_params: MPDrawParams) -> None:
    """Draw scenario from config with traffic signs.

    If requested by the debug config, also draw planning problem and reference path.
    """
    scenario = config.scenario
    scenario.draw(renderer, draw_params)
    for sign in scenario.lanelet_network.traffic_signs:
        sign.draw(renderer)

    if config.debug.draw_planning_problem:
        config.planning_problem.draw(renderer, draw_params)

    ref_path = config.planning.reference_path
    if config.debug.draw_ref_path and ref_path is not None:
        renderer.ax.plot(ref_path[:, 0], ref_path[:, 1], color='g', marker='.', markersize=1, zorder=19, linewidth=2.0)
