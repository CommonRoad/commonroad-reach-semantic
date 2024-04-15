import copy
from typing import List, Optional, Tuple

import numpy as np
import seaborn as sns
from commonroad.geometry.shape import Polygon
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.scenario import Scenario
from commonroad.visualization.draw_params import MPDrawParams
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad_dc.geometry.geometry import CurvilinearCoordinateSystem
from commonroad_reach.data_structure.reach.reach_node import ReachNode
from matplotlib import pyplot as plt


def plot_exid(
        scenario: Scenario,
        planning_problem: PlanningProblem,
        ref_path: Optional[np.ndarray] = None,
        plot_limits: Optional[List[float]] = None,
        figsize: Optional[Tuple[float, float]] = None,
        draw_trajectories_for_ids: Optional[List[int]] = None,
        time_step: int = 0,
) -> MPRenderer:
    draw_params = _create_draw_params()
    draw_params.time_begin = time_step

    renderer = MPRenderer(plot_limits=plot_limits, figsize=figsize, draw_params=draw_params)

    scenario.draw(renderer, draw_params)

    draw_trajectories_for_ids = draw_trajectories_for_ids or []
    for obs_id in draw_trajectories_for_ids:
        obs = scenario.obstacle_by_id(obs_id)
        obs.prediction.trajectory.draw(renderer, draw_params)

    planning_problem.draw(renderer, draw_params)

    if ref_path is not None:
        ref_path_params = copy.deepcopy(draw_params.shape)
        ref_path_params.facecolor = "#ff477e"
        ref_path_params.edgecolor = ref_path_params.facecolor
        ref_path_params.linewidth = 2
        ref_path_params.zorder -= 1
        draw_path = np.concatenate((ref_path, np.flip(ref_path, axis=0)))
        Polygon(draw_path).draw(renderer, ref_path_params)

    # settings and adjustments
    plt.rc("axes", axisbelow=True)
    ax = plt.gca()
    ax.set_aspect("equal")
    plt.margins(0, 0)

    return renderer


def _create_draw_params() -> MPDrawParams:
    draw_params = MPDrawParams()

    draw_params.dynamic_obstacle.draw_icon = True
    draw_params.dynamic_obstacle.trajectory.draw_trajectory = True
    draw_params.dynamic_obstacle.occupancy.draw_occupancies = False

    palette = sns.color_palette("GnBu_d", 3)
    edge_color = (palette[0][0] * 0.75, palette[0][1] * 0.75, palette[0][2] * 0.75)
    draw_params.shape.facecolor = palette[0]
    draw_params.shape.edgecolor = edge_color

    draw_params.planning_problem.initial_state.state.radius = 0.5
    draw_params.planning_problem.initial_state.state.draw_arrow = True
    draw_params.planning_problem.initial_state.state.arrow.width = 0.5

    draw_params.time_begin = 0
    draw_params.dynamic_obstacle.trajectory.draw_trajectory = False

    return draw_params
