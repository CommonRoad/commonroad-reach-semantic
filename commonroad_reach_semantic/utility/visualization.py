import copy
import logging
from pathlib import Path
from typing import List, Tuple, Union, Set, Dict

import commonroad_reach.utility.logger as util_logger
import commonroad_reach.utility.visualization as reach_visualization
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
# from commonroad_reach_semantic import pycrreachs as reach
from commonroad.geometry.shape import Polygon
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.visualization.draw_params import MPDrawParams
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad_reach.data_structure.reach.reach_interface import ReachableSetInterface
from commonroad_reach.utility import coordinate_system as util_coordinate_system

from commonroad_reach_semantic.data_structure.driving_corridor_extractor import DrivingCorridor
from commonroad_reach_semantic.data_structure.model_checking.kripke import KripkeNode
from commonroad_reach_semantic.data_structure.config.semantic_configuration import SemanticConfiguration
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.model_checking.spot_interface import SpotInterface

logger = logging.getLogger(__name__)
logging.getLogger('PIL').setLevel(logging.WARNING)
logging.getLogger('matplotlib.font_manager').setLevel(logging.WARNING)


class ColorMapper:
    def __init__(self, reach_interface: ReachableSetInterface, steps: List[int]):
        # determine number of colors

        different_propositions = {
            frozenset(reach_interface._reach.labeler.reachable_set_to_propositions[reach_node].set_propositions)
            for step in steps
            for reach_node in reach_interface.reachable_set_at_step(step)
        }

        self.num_colors = len(different_propositions)
        self.palette = sns.color_palette("rainbow", self.num_colors)

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


def plot_scenario_with_reachable_sets(reach_interface: ReachableSetInterface, figsize: Tuple = None,
                                      step_start: int = 0, step_end: int = 0, steps: List[int] = None,
                                      plot_limits: List = None, path_output: str = None,
                                      save_gif: bool = True, duration: float = None):
    """
    Plots scenario with computed reachable sets.
    """
    config: SemanticConfiguration = reach_interface.config
    scenario = config.scenario
    planning_problem = config.planning_problem
    ref_path = config.planning.reference_path

    path_output = path_output or config.general.path_output
    Path(path_output).mkdir(parents=True, exist_ok=True)

    figsize = figsize if figsize else (25, 15)
    plot_limits = plot_limits or compute_plot_limits_from_reachable_sets(reach_interface)
    palette = sns.color_palette("GnBu_d", 3)
    edge_color = (palette[0][0] * 0.75, palette[0][1] * 0.75, palette[0][2] * 0.75)

    # generate default drawing parameters
    draw_params = reach_visualization.generate_default_drawing_parameters(config)
    draw_params.shape.facecolor = palette[0]
    draw_params.shape.edgecolor = edge_color

    step_start = step_start or reach_interface.step_start
    step_end = step_end or reach_interface.step_end
    if steps:
        steps = [step for step in steps if step <= step_end + 1]
    else:
        # add additional step_start for dirty fixing an issue in plotting the traffic signs
        steps = [step_start] + list(range(step_start, step_end + 1))
    duration = duration if duration else config.planning.dt

    mapper = ColorMapper(reach_interface, steps)

    util_logger.print_and_log_info(logger, "* Plotting reachable sets...")
    renderer = MPRenderer(plot_limits=plot_limits, figsize=figsize) if config.debug.save_plots else None
    for step in steps:
        time_step = step * round(config.planning.dt / config.scenario.dt)
        if config.debug.save_plots:
            # clear previous plot
            plt.cla()
        else:
            # create new figure
            plt.figure(figsize=figsize)
            renderer = MPRenderer(plot_limits=plot_limits)

        # plot scenario and planning problem
        draw_params.time_begin = time_step
        scenario.draw(renderer, draw_params)

        if config.debug.draw_planning_problem:
            planning_problem.draw(renderer, draw_params)

        dict_nodes_reach = reach_interface.reachable_set_at_step(step)
        draw_reachable_sets(dict_nodes_reach, config, renderer, draw_params, mapper, reach_interface)

        # plot traffic signs
        for sign in scenario.lanelet_network.traffic_signs:
            sign.draw(renderer)

        # plot reference path
        if config.debug.draw_ref_path and ref_path is not None:
            renderer.ax.plot(ref_path[:, 0], ref_path[:, 1],
                             color='g', marker='.', markersize=1, zorder=19, linewidth=2.0)

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
                               figsize: Tuple = None, plot_limits: Union[List] = None, path_output: str = None):
    """
    Plots scenario with computed lanelet regions.
    """
    config: SemanticConfiguration = semantic_model.config
    scenario = config.scenario

    path_output = path_output or config.general.path_output
    Path(path_output).mkdir(parents=True, exist_ok=True)

    figsize = figsize if figsize else (25, 15)
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
                                    figsize: Tuple = None,
                                    step_start: int = 0, step_end: int = 0, steps: List[int] = None,
                                    plot_limits: Union[List] = None, path_output: str = None,
                                    save_gif: bool = True, duration: float = None):
    """
    Plots scenario with accepting kripke nodes.
    """
    config: SemanticConfiguration = spot_interface.reach_interface.config
    reach_interface = spot_interface.reach_interface
    scenario = config.scenario
    planning_problem = config.planning_problem
    ref_path = config.planning.reference_path

    path_output = path_output or config.general.path_output
    Path(path_output).mkdir(parents=True, exist_ok=True)

    figsize = figsize if figsize else (25, 15)
    plot_limits = plot_limits or compute_plot_limits_from_reachable_sets(reach_interface)
    palette = sns.color_palette("GnBu_d", 3)
    edge_color = (palette[0][0] * 0.75, palette[0][1] * 0.75, palette[0][2] * 0.75)

    # generate default drawing parameters
    draw_params = reach_visualization.generate_default_drawing_parameters(config)
    draw_params.shape.facecolor = palette[0]
    draw_params.shape.edgecolor = edge_color

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

    mapper = ColorMapper(reach_interface, steps)

    util_logger.print_and_log_info(logger, "* Plotting reachable sets in accepting kripke nodes...")
    renderer = MPRenderer(plot_limits=plot_limits, figsize=figsize) if config.debug.save_plots else None
    for step in steps:
        time_step = step * round(config.planning.dt / config.scenario.dt)
        if config.debug.save_plots:
            # clear previous plot
            plt.cla()
        else:
            # create new figure
            plt.figure(figsize=figsize)
            renderer = MPRenderer(plot_limits=plot_limits)

        # plot scenario and planning problem
        draw_params.time_begin = time_step
        scenario.draw(renderer, draw_params)

        if config.debug.draw_planning_problem:
            planning_problem.draw(renderer, draw_params)

        draw_kripke_nodes(dict_step_to_set_nodes_kripke[step], config, renderer, draw_params, mapper, reach_interface)

        # plot traffic signs
        for sign in scenario.lanelet_network.traffic_signs:
            sign.draw(renderer)

        # plot reference path
        if config.debug.draw_ref_path and ref_path is not None:
            renderer.ax.plot(ref_path[:, 0], ref_path[:, 1],
                             color='g', marker='.', markersize=1, zorder=19, linewidth=2.0)

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


def plot_scenario_with_driving_corridor(spot_interface: SpotInterface, corridor: DrivingCorridor, figsize: Tuple = None,
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
    scenario = config.scenario
    planning_problem = config.planning_problem
    ref_path = config.planning.reference_path

    path_output = path_output or config.general.path_output
    Path(path_output).mkdir(parents=True, exist_ok=True)

    figsize = figsize if figsize else (25, 15)
    plot_limits = plot_limits or compute_plot_limits_from_reachable_sets(reach_interface)
    palette = sns.color_palette("GnBu_d", 3)
    edge_color = (palette[0][0] * 0.75, palette[0][1] * 0.75, palette[0][2] * 0.75)

    # generate default drawing parameters
    draw_params = reach_visualization.generate_default_drawing_parameters(config)
    draw_params.shape.facecolor = palette[0]
    draw_params.shape.edgecolor = edge_color

    step_start = step_start or reach_interface.step_start
    step_end = step_end or reach_interface.step_end
    if steps:
        steps = [step for step in steps if step <= step_end + 1]
    else:
        # add additional step_start for dirty fixing an issue in plotting the traffic signs
        steps = [step_start] + list(range(step_start, step_end + 1))
    duration = duration if duration else config.planning.dt

    dict_step_to_set_nodes_kripke = corridor.retrieve_kripke_nodes()

    mapper = ColorMapper(reach_interface, steps)

    util_logger.print_and_log_info(logger, "* Plotting driving corridor...")
    renderer = MPRenderer(plot_limits=plot_limits, figsize=figsize) if config.debug.save_plots else None
    for step in steps:
        time_step = step * round(config.planning.dt / config.scenario.dt)
        if config.debug.save_plots:
            # clear previous plot
            plt.cla()
        else:
            # create new figure
            plt.figure(figsize=figsize)
            renderer = MPRenderer(plot_limits=plot_limits)

        # plot scenario and planning problem
        draw_params.time_begin = time_step
        scenario.draw(renderer, draw_params)

        if config.debug.draw_planning_problem:
            planning_problem.draw(renderer, draw_params)

        set_nodes_reach = set()
        for node_kripke in dict_step_to_set_nodes_kripke[step]:
            set_nodes_reach.update(node_kripke.set_nodes_reach)
        draw_reachable_sets(set_nodes_reach, config, renderer, draw_params, mapper, reach_interface)

        # plot traffic signs
        for sign in scenario.lanelet_network.traffic_signs:
            sign.draw(renderer)

        # plot reference path
        if config.debug.draw_ref_path and ref_path is not None:
            renderer.ax.plot(ref_path[:, 0], ref_path[:, 1],
                             color='g', marker='.', markersize=1, zorder=19, linewidth=2.0)

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


def compute_plot_limits_from_reachable_sets(reach_interface: ReachableSetInterface, margin: int = 20):
    """
    Returns plot limits from the computed reachable sets.

    :param reach_interface: interface holding the computed reachable sets.
    :param margin: additional margin for the plot limits.
    :return:
    """
    config: SemanticConfiguration = reach_interface.config
    x_min = y_min = np.infty
    x_max = y_max = -np.infty
    backend = "CPP" if config.reachable_set.mode_computation == 2 else "PYTHON"
    coordinate_system = config.planning.coordinate_system

    if coordinate_system == "CART":
        for step in range(reach_interface.step_start, reach_interface.step_end):
            for rectangle in reach_interface.drivable_area_at_step(step):
                bounds = rectangle.bounds if backend == "PYTHON" else (rectangle.p_lon_min(), rectangle.p_lat_min(),
                                                                       rectangle.p_lon_max(), rectangle.p_lat_max())
                x_min = min(x_min, bounds[0])
                y_min = min(y_min, bounds[1])
                x_max = max(x_max, bounds[2])
                y_max = max(y_max, bounds[3])

    elif config.planning.coordinate_system == "CVLN":
        for step in range(reach_interface.step_start, reach_interface.step_end):
            for rectangle_cvln in reach_interface.drivable_area_at_step(step):
                list_rectangles_cart = util_coordinate_system.convert_to_cartesian_polygons(rectangle_cvln,
                                                                                            config.planning.CLCS, False)
                for rectangle_cart in list_rectangles_cart:
                    bounds = rectangle_cart.bounds
                    x_min = min(x_min, bounds[0])
                    y_min = min(y_min, bounds[1])
                    x_max = max(x_max, bounds[2])
                    y_max = max(y_max, bounds[3])

    if np.inf in (x_min, y_min) or -np.inf in (x_max, y_max):
        return None

    else:
        return [x_min - margin, x_max + margin, y_min - margin, y_max + margin]


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


def draw_reachable_sets(nodes, config: SemanticConfiguration, renderer, draw_params, mapper: ColorMapper, reach_interface: ReachableSetInterface):
    backend = "CPP" if config.reachable_set.mode_computation == 2 else "PYTHON"
    coordinate_system = config.planning.coordinate_system

    if coordinate_system == "CART":
        for node in nodes:
            vertices = node.position_rectangle.vertices if backend == "PYTHON" else node.position_rectangle().vertices()
            draw_params.shape.facecolor = mapper.map_to_color(reach_interface._reach.labeler.reachable_set_to_propositions[node].set_propositions)
            Polygon(vertices=np.array(vertices)).draw(renderer, draw_params)

    elif coordinate_system == "CVLN":
        for node in nodes:
            position_rectangle = node.position_rectangle if backend == "PYTHON" else node.position_rectangle()
            list_polygons_cart = util_coordinate_system.convert_to_cartesian_polygons(position_rectangle,
                                                                                      config.planning.CLCS, True)
            draw_params.shape.facecolor = mapper.map_to_color(reach_interface._reach.labeler.reachable_set_to_propositions[node].set_propositions)
            for polygon in list_polygons_cart:
                Polygon(vertices=np.array(polygon.vertices)).draw(renderer, draw_params)


def draw_kripke_nodes(set_nodes_kripke: Set[KripkeNode], config: SemanticConfiguration, renderer,
                      draw_params: MPDrawParams,
                      mapper: ColorMapper,
                      reach_interface: ReachableSetInterface):
    backend = "CPP" if config.reachable_set.mode_computation == 2 else "PYTHON"
    coordinate_system = config.planning.coordinate_system

    for node_kripke in set_nodes_kripke:
        draw_params_nodes = copy.deepcopy(draw_params)

        if coordinate_system == "CART":
            for node in node_kripke.set_nodes_reach:
                vertices = node.position_rectangle.vertices if backend == "PYTHON" \
                    else node.position_rectangle().vertices()
                draw_params.shape.facecolor = mapper.map_to_color(reach_interface._reach.labeler.reachable_set_to_propositions[node].set_propositions)
                Polygon(vertices=np.array(vertices)).draw(renderer, draw_params_nodes)

        elif coordinate_system == "CVLN":
            for node in node_kripke.set_nodes_reach:
                position_rectangle = node.position_rectangle if backend == "PYTHON" else node.position_rectangle()
                list_polygons_cart = util_coordinate_system.convert_to_cartesian_polygons(position_rectangle,
                                                                                          config.planning.CLCS, True)
                draw_params_nodes.shape.facecolor = mapper.map_to_color(reach_interface._reach.labeler.reachable_set_to_propositions[node].set_propositions)
                for polygon in list_polygons_cart:
                    Polygon(vertices=np.array(polygon.vertices)).draw(renderer, draw_params_nodes)


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
            raise Exception("Coordinate system not defined.")

        if list_vertices_cart:
            list_vertices_cart.append(list_vertices_cart[0])

            idx_palette += 1
            draw_params = MPDrawParams()
            draw_params.shape.facecolor = palette[idx_palette % num_colors]
            draw_params.shape.edgecolor = palette[idx_palette % num_colors]
            draw_params.shape.zorder = 10

            Polygon(vertices=np.array(list_vertices_cart)).draw(renderer, draw_params)
