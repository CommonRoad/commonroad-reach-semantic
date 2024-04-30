from collections import defaultdict

import commonroad_reach.utility.logger as util_logger

from commonroad_reach_semantic.data_structure.config.semantic_configuration_builder import SemanticConfigurationBuilder
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.model_checking.spot_interface import SpotInterface
from commonroad_reach_semantic.data_structure.reach.semantic_reach_interface import SemanticReachableSetInterface
from commonroad_reach_semantic.data_structure.rule.traffic_rule_interface import TrafficRuleInterface
from commonroad_reach_semantic.utility import visualization as util_visual
from commonroad.scenario.state import InitialState


def main():
    # ==== specify scenario
    name_scenario = "DEU_TestRIN1-3_1_T-1"
    # name_scenario = "ZAM_Intersection-1_2_T-1"
    #name_scenario = "ZAM_Merge-1_1_T-1"
    # name_scenario = "ZAM_Yield-1_1_T-1"
    # name_scenario = "ESP_Monzon-2_2_T-1"
    #name_scenario = "USA_US101-6_1_T-1"
    # name_scenario = "ZAM_Over-1_1"
    # name_scenario = "DEU_Gar-1_1_T-1"

    # ==== build configuration
    path_root = "/home/liny/repairverse/commonroad-reach-semantic"
    config = SemanticConfigurationBuilder(path_root=path_root).build_configuration(name_scenario)

    config.update()
    # config.planning_problem.initial_state.time_step = 5
    util_logger.initialize_logger(config)
    config.print_configuration_summary()

    from commonroad.visualization.mp_renderer import MPRenderer
    rnd = MPRenderer()
    rnd.draw_params.time_begin = 24
    config.scenario.draw(rnd)
    config.scenario.obstacle_by_id(
        31
    ).draw(rnd)
    rnd.render()
    import matplotlib.pyplot as plt
    plt.show()

    # target_veh = config.scenario.obstacle_by_id(200)
    # config.scenario.remove_obstacle(target_veh)
    # target_state = target_veh.state_at_time(10)
    # config.planning_problem.initial_state = InitialState(
    #     position=target_state.position,
    #     velocity=target_state.velocity,
    #     time_step=target_state.time_step,
    #     yaw_rate=0,
    #     slip_angle=0,
    #     orientation=target_state.orientation
    # )
    # config.scenario.remove_obstacle(config.scenario.obstacles)
    # config.planning_problem.initial_state.position = [5, 0]
    config.vehicle.ego.length = 5
    # from commonroad_reach.utility import configuration as util_configuration
    #
    # config.vehicle.ego.radius_disc, config.vehicle.ego.circle_distance = \
    #     util_configuration.compute_disc_radius_and_distance(config.vehicle.ego.length, config.vehicle.ego.width,
    #                                                         ref_point=config.planning.reference_point,
    #                                                         dist_axle_rear=config.vehicle.ego.wb_rear_axle)

    # config.vehicle.ego.radius_inflation = util_configuration.compute_inflation_radius(config.reachable_set.mode_inflation,
    #                                                                config.vehicle.ego.length, config.vehicle.ego.width,
    #                                                                config.vehicle.ego.radius_disc)

    config.planning_problem.initial_state.time_step = 23
    # config.update()

    config.reachable_set.mode_computation = 8

    # ==== initialize semantic model and traffic rules
    semantic_model = SemanticModel(config)
    rule_interface = TrafficRuleInterface(config, semantic_model)
    rule_interface.print_summary()
    reach_interface = SemanticReachableSetInterface(config, semantic_model, rule_interface)
    # config.vehicle.ego.wb_rear_axle = 2.0

    config.planning.steps_computation = 15
    config.planning_problem.initial_state.position = [17.2906,  0.]
    config.planning_problem.initial_state.velocity = 3.1875
    config.planning_problem.initial_state.acceleration = -1.875
    # config.scenario.remove_obstacle(config.scenario.obstacles)
    config.vehicle.ego.v_lon_min = 0
    # ==== compute reachable sets using reachability interface
    config.update(
        scenario=config.scenario,
        planning_problem=config.planning_problem,
        CLCS=config.planning.CLCS,
    )
    rule_interface.list_traffic_rules_activated = ["LTL G[1..30](BehindStopLine)"]
    for item in rule_interface.list_traffic_rules_activated:
        rule_interface._parse_traffic_rule(item, allow_abstract_rules=True)

    reach_interface.reset(config=config)

    reach_interface.compute_reachable_sets()


    # ==== plot computation results
    if config.reachable_set.mode_computation in [5, 6]:
        node_to_group = util_visual.groups_from_propositions(
            reach_interface._reach.labeler.reachable_set_to_propositions)
    elif config.reachable_set.mode_computation in [7, 8]:
        node_to_group = util_visual.groups_from_states(reach_interface._reach.reachable_set_to_label)
    else:
        # no semantic information, so put all nodes in the same group
        node_to_group = defaultdict(lambda: 0)

    util_visual.plot_reach_graph(reach_interface, node_to_group=node_to_group)
    util_visual.plot_scenario_with_regions(semantic_model, "CVLN")
    util_visual.plot_scenario_with_reachable_sets(reach_interface, save_gif=True, plot_limits=[0, 55, -25, 25])

    # # ==== show interactive visualization (can take a long time to plot if there are many nodes)
    #util_visual.show_interactive_reach_graph(reach_interface, use_images=True, node_to_group=node_to_group)

    if config.reachable_set.mode_computation in [5, 6]:
        # only necessary for labeling reachable set
        spot_interface = SpotInterface(reach_interface, rule_interface)
        spot_interface.translate_ltl_formulas()
        spot_interface.translate_reachability_graph()
        spot_interface.check()
        util_visual.plot_scenario_with_kripke_nodes(spot_interface, plot_accepting=True, save_gif=True)


if __name__ == "__main__":
    main()
