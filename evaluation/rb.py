from time import time
import matplotlib
matplotlib.use('TkAgg')

# commonroad
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.mp_renderer import MPRenderer

#commonroad_dc
from commonroad_dc.boundary import boundary
from commonroad_dc.collision.trajectory_queries import trajectory_queries
from commonroad_dc.collision.collision_detection.pycrcc_collision_dispatch import create_collision_checker
import commonroad_dc.pycrcc as pycrcc
from commonroad_dc.pycrcc.Util import trajectory_enclosure_polygons_static
from matplotlib import pyplot as plt


def open_scenario(scenario_filename):
    crfr = CommonRoadFileReader(
        scenario_filename)
    scenario, planning_problem_set = crfr.open()
    return scenario, planning_problem_set

#open the example scenario
scenario, planning_problem_set = open_scenario("/home/lercher/tum/commonroad/dataset-converters/commonroad_dataset_converter/datasets/exiD/maps/6_merzenich_rather.xml")

# # plot the scenario
# rnd = MPRenderer(figsize=(25, 10))
# scenario.draw(rnd)
# planning_problem_set.draw(rnd)
# rnd.render()

time1=time()
road_boundary_obstacle, road_boundary_sg_triangles=boundary.create_road_boundary_obstacle(scenario, method='triangulation')
time2=time()

print("Computation time: %s" % (time2-time1))

# draw the road boundary
rnd = MPRenderer(figsize=(25, 10))
road_boundary_sg_triangles.draw(rnd)
rnd.render()
print("Number of boundary elements: %s" % road_boundary_sg_triangles.size())
plt.show(block=True)
