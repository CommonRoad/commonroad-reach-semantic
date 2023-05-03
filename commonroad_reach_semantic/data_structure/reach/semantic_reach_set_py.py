import itertools
import logging
from collections import defaultdict
from typing import List

import commonroad_reach.utility.logger as util_logger
from commonroad_reach.data_structure.collision_checker import CollisionChecker
from commonroad_reach.data_structure.reach.reach_polygon import ReachPolygon
from commonroad_reach.utility import reach_operation

import commonroad_reach_semantic.utility.reach_operation as semantic_reach_operation
from commonroad_reach_semantic.data_structure.rule.proposition import PropositionGroup as PropGroup
from commonroad_reach_semantic.data_structure.rule.proposition_holder import PropositionHolder
from commonroad_reach_semantic.data_structure.reach.semantic_reach_node import SemanticReachNode
from commonroad_reach_semantic.data_structure.reach.semantic_reach_set import SemanticReachableSet
from commonroad_reach_semantic.data_structure.config.semantic_configuration import SemanticConfiguration
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.rule.traffic_rule_interface import TrafficRuleInterface

logger = logging.getLogger(__name__)


class PySemanticReachableSet(SemanticReachableSet):
    """
    Reachable set computation considering temporal constraints with Python backend.
    """

    config: SemanticConfiguration

    def __init__(self, config: SemanticConfiguration, semantic_model: SemanticModel,
                 rule_interface: TrafficRuleInterface):
        super().__init__(config, semantic_model, rule_interface)
        self.dict_step_to_reachable_set[self.step_start] = self._construct_initial_reachable_set()
        self.dict_step_to_drivable_area[self.step_start] = reach_operation.project_propagated_sets_to_position_domain(
            self.dict_step_to_reachable_set[self.step_start])

        self.dict_step_to_propositions_to_drivable_area = dict()
        self.dict_step_to_propositions_to_propagated_set = dict()

        self._label_initial_state()
        self._initialize_zero_state_polygons()
        self.collision_checker = CollisionChecker(self.config)

        logger.debug("PySemanticReachableSet initialized.")

    def _construct_initial_reachable_set(self) -> List[SemanticReachNode]:
        tuple_vertices_polygon_lon, tuple_vertices_polygon_lat = \
            reach_operation.generate_tuples_vertices_polygons_initial(self.config)

        polygon_lon = ReachPolygon.from_rectangle_vertices(*tuple_vertices_polygon_lon)
        polygon_lat = ReachPolygon.from_rectangle_vertices(*tuple_vertices_polygon_lat)

        return [SemanticReachNode(polygon_lon, polygon_lat, self.config.planning.step_start)]

    def _label_initial_state(self):
        """
        Assigns proposition labels to initial reachable sets and drivable areas.
        """
        for drivable_area, reachable_set in zip(self.dict_step_to_drivable_area[self.step_start],
                                                self.dict_step_to_reachable_set[self.step_start]):
            propositions = self._obtain_propositions_for_rectangle(drivable_area, self.step_start)
            reachable_set.proposition_holder.merge(propositions)
        self.semantic_model.label_traffic_propositions(self.step_start,
                                                       self.dict_step_to_reachable_set[self.step_start])

    def _obtain_propositions_for_rectangle(self, rectangle: ReachPolygon, step: int) -> PropositionHolder:
        """
        Returns the propositions of the given rectangle.

        Intersects the rectangle with regions and position intervals.
        """
        proposition_holder = PropositionHolder()
        # retrieve propositions from the intersecting lanelet region
        for region in self.semantic_model.region_model.list_regions:
            if region.polygon_cvln.intersects(rectangle):
                for group, set_propositions in region.dict_group_to_propositions_at_step(step).items():
                    proposition_holder.add_propositions(set_propositions, group)
                break

        # retrieve vehicle-related propositions from position intervals
        list_intervals_lon = self.semantic_model.vehicle_model.dict_step_to_position_intervals[step]["lon"]
        list_intervals_lat = self.semantic_model.vehicle_model.dict_step_to_position_intervals[step]["lat"]

        for interval_lon in list_intervals_lon:
            if interval_lon.intersects(rectangle.p_lon_min, rectangle.p_lon_max):
                proposition_holder.add_propositions(interval_lon.set_propositions, PropGroup.POSITION)
                break

        for interval_lat in list_intervals_lat:
            if interval_lat.intersects(rectangle.p_lat_min, rectangle.p_lat_max):
                proposition_holder.add_propositions(interval_lat.set_propositions, PropGroup.POSITION)
                break

        return proposition_holder

    def _initialize_zero_state_polygons(self):
        """
        Initializes the zero-state polygons of the system.

        Computation of the reachable set of an LTI system requires the zero-state response of the system.
        """
        self.polygon_zero_state_lon = reach_operation.create_zero_state_polygon(self.config.planning.dt,
                                                                                self.config.vehicle.ego.a_lon_min,
                                                                                self.config.vehicle.ego.a_lon_max)

        self.polygon_zero_state_lat = reach_operation.create_zero_state_polygon(self.config.planning.dt,
                                                                                self.config.vehicle.ego.a_lat_min,
                                                                                self.config.vehicle.ego.a_lat_max)

    def compute(self, step_start: int, step_end: int):
        for step in range(step_start, step_end + 1):
            logger.debug(f"Computing reachable set for step {step}")
            self._compute_drivable_area_at_step(step)
            self._compute_reachable_set_at_step(step)
            self._list_steps_computed.append(step)

        if self.config.reachable_set.prune_nodes_not_reaching_final_step:
            self.prune_nodes_not_reaching_final_step()

    def compute_drivable_area_at_step(self, step):
        logger.debug(f"Computing drivable area for step {step}")
        self._compute_drivable_area_at_step(step)

        if step not in self._list_steps_computed:
            self._list_steps_computed.append(step)

    def compute_reachable_set_at_step(self, step):
        logger.debug(f"Computing reachable set for step {step}")
        self._compute_reachable_set_at_step(step)
        if step not in self._list_steps_computed:
            self._list_steps_computed.append(step)

    def _compute_drivable_area_at_step(self, step: int):
        """
        Computes drivable area for the given step.

        Steps:
            1. Propagate each node of the reachable set from the previous step, resulting in propagated base sets.
            2. Split and label propagated sets according to relevant atomic propositions.
            2. Project base sets onto the position domain to obtain position rectangles.
            3. Merge, repartition and check collisions for these rectangles. The order depends on the configuration.
        """
        reachable_set_previous = self.dict_step_to_reachable_set[step - 1]

        if len(reachable_set_previous) < 1:
            self.dict_step_to_drivable_area[step] = list()
            self.dict_step_to_propagated_set[step] = list()
            return None

        propagated_sets = self._propagate_reachable_set(reachable_set_previous)

        # split w.r.t regions and position intervals
        propagated_sets = itertools.chain.from_iterable(
            self.semantic_model.split_wrt_regions(step, propagated_set) for propagated_set in propagated_sets)
        propagated_sets = itertools.chain.from_iterable(
            self.semantic_model.split_wrt_position_intervals(step, propagated_set) for propagated_set in
            propagated_sets)

        # discard the ones colliding with vehicles
        propagated_sets = (propagated_set for propagated_set in propagated_sets if
                           not propagated_set.collides_with_vehicle())

        # examine whether the propagated sets satisfy TPL specifications
        propagated_sets = self.rule_interface.tpl_checker.examine_tpl_specifications(step, list(propagated_sets))

        # update traffic propositions of the propagated sets
        propagated_sets = self.semantic_model.label_traffic_propositions(step, propagated_sets)

        # partition propagated sets by their propositions
        dict_propositions_to_propagated_set = defaultdict(list)
        for propagated_set in propagated_sets:
            dict_propositions_to_propagated_set[propagated_set.proposition_holder].append(propagated_set)

        # merge, collision check, and repartition propagated sets partitioned by their propositions,
        # because we must not merge sets with different propositions
        dict_propositions_to_drivable_area = dict()
        for propositions, propagated_sets_per_proposition in dict_propositions_to_propagated_set.items():
            list_rectangles_projected = reach_operation.project_propagated_sets_to_position_domain(
                propagated_sets_per_proposition)
            dict_propositions_to_drivable_area[propositions] = self._collision_check_and_repartition(
                list_rectangles_projected, step)

        self.dict_step_to_drivable_area[step] = list(
            itertools.chain.from_iterable(dict_propositions_to_drivable_area.values()))
        self.dict_step_to_propositions_to_drivable_area[step] = dict_propositions_to_drivable_area
        self.dict_step_to_propositions_to_propagated_set[step] = dict_propositions_to_propagated_set
        self.dict_step_to_propagated_set[step] = propagated_sets

    def _propagate_reachable_set(self, list_nodes: List[SemanticReachNode]) -> List[SemanticReachNode]:
        """
        Propagates nodes of the reachable set.
        """
        # TODO: set propagation constraints
        no_backward_driving = "NoBackwardDrivingRule" in self.rule_interface.list_traffic_rules_activated
        v_lon_min = max(0, self.config.vehicle.ego.v_lon_min) if no_backward_driving \
            else self.config.vehicle.ego.v_lon_min

        list_base_sets_propagated = []
        for node in list_nodes:
            try:
                # propagate in both directions
                polygon_lon_propagated = reach_operation.propagate_polygon(node.polygon_lon,
                                                                           self.polygon_zero_state_lon,
                                                                           self.config.planning.dt,
                                                                           v_lon_min,
                                                                           self.config.vehicle.ego.v_lon_max)

                polygon_lat_propagated = reach_operation.propagate_polygon(node.polygon_lat,
                                                                           self.polygon_zero_state_lat,
                                                                           self.config.planning.dt,
                                                                           self.config.vehicle.ego.v_lat_min,
                                                                           self.config.vehicle.ego.v_lat_max)
            except (ValueError, RuntimeError, AttributeError):
                util_logger.print_and_log_debug(logger, "Error occurred while propagating polygons.")

            else:
                base_set_propagated = SemanticReachNode(polygon_lon_propagated, polygon_lat_propagated, node.step)
                base_set_propagated.source_propagation = node
                list_base_sets_propagated.append(base_set_propagated)

        return list_base_sets_propagated

    def _collision_check_and_repartition(self, rectangles: List[ReachPolygon], step: int) -> List[ReachPolygon]:
        mode_repartition = self.config.reachable_set.mode_repartition
        size_grid = self.config.reachable_set.size_grid
        size_grid_2nd = self.config.reachable_set.size_grid_2nd
        radius_terminal_split = self.config.reachable_set.radius_terminal_split

        # repartition, then collision check
        if mode_repartition == 1:
            list_rectangles_repartitioned = \
                reach_operation.create_repartitioned_rectangles(rectangles, size_grid)
            rectangles = reach_operation.check_collision_and_split_rectangles(self.collision_checker, step,
                                                                              list_rectangles_repartitioned,
                                                                              radius_terminal_split)

        # collision check, then repartition
        elif mode_repartition == 2:
            list_rectangles_collision_free = \
                reach_operation.check_collision_and_split_rectangles(self.collision_checker, step,
                                                                     rectangles,
                                                                     radius_terminal_split)
            rectangles = reach_operation.create_repartitioned_rectangles(list_rectangles_collision_free,
                                                                         size_grid)

        # repartition, collision check, then repartition again
        elif mode_repartition == 3:
            list_rectangles_repartitioned = reach_operation.create_repartitioned_rectangles(rectangles,
                                                                                            size_grid)

            list_rectangles_collision_free = \
                reach_operation.check_collision_and_split_rectangles(self.collision_checker, step,
                                                                     list_rectangles_repartitioned,
                                                                     radius_terminal_split)

            rectangles = reach_operation.create_repartitioned_rectangles(list_rectangles_collision_free,
                                                                         size_grid_2nd)

        else:
            raise Exception("Invalid mode for repartition.")

        return rectangles

    def _compute_reachable_set_at_step(self, step):
        """
        Computes reachable set for the given step.

        Steps:
            1. construct reach nodes from drivable area and the propagated sets.
            2. update parent-child relationship of the nodes.
        """
        dict_propositions_to_propagated_set = self.dict_step_to_propositions_to_propagated_set[step]
        dict_propositions_to_drivable_area = self.dict_step_to_propositions_to_drivable_area[step]

        if not dict_propositions_to_drivable_area:
            self.dict_step_to_reachable_set[step] = list()
            return None

        # discard drivable area with small area if there are more than one node (this is subject to change)
        num_drivable_area = sum(
            [len(list_drivable) for list_drivable in dict_propositions_to_drivable_area.values()])
        discard_small_node = (num_drivable_area > 1)

        # work with the reachable sets partitioned by propositions here, because otherwise it could happen
        # that we merge two reachable sets with different propositions when they intersect with the same drivable area
        dict_propositions_to_reachable_set = dict()
        for proposition_holder, drivable_area in dict_propositions_to_drivable_area.items():
            propagated_set = dict_propositions_to_propagated_set[proposition_holder]

            list_nodes = semantic_reach_operation.construct_reach_nodes(drivable_area, propagated_set)
            if discard_small_node:
                list_nodes = semantic_reach_operation.discard_nodes_with_short_edge(list_nodes,
                                                                                    self.config.reachable_set.length_edge_node_min)
            if list_nodes:
                reachable_set = reach_operation.connect_children_to_parents(step, list_nodes)
                dict_propositions_to_reachable_set[proposition_holder] = reachable_set

        self.dict_step_to_reachable_set[step] = list(
            itertools.chain.from_iterable(dict_propositions_to_reachable_set.values()))

    def _reset_reachable_set_at_step(self, step: int, reachable_set: List[SemanticReachNode]):
        reachable_set_cur: List[SemanticReachNode] = self.dict_step_to_reachable_set[step]
        for node in reachable_set_cur:
            for node_parent in node.list_nodes_parent:
                node_parent.remove_child_node(node)

        self.dict_step_to_reachable_set[step] = reachable_set

    def prune_nodes_not_reaching_final_step(self):
        util_logger.print_and_log_info(logger, f"\tPruning nodes not reaching final step...")
        cnt_nodes_before_pruning = cnt_nodes_after_pruning = len(self.reachable_set_at_step(self.step_end))

        for step in range(self.step_end - 1, self.step_start - 1, -1):
            list_nodes = self.reachable_set_at_step(step)
            cnt_nodes_before_pruning += len(list_nodes)

            list_idx_nodes_to_be_deleted = list()
            for idx_node, node in enumerate(list_nodes):
                # discard the node if it has no child node
                if not node.list_nodes_child:
                    list_idx_nodes_to_be_deleted.append(idx_node)
                    # iterate through its parent nodes and disconnect them
                    for node_parent in node.list_nodes_parent:
                        node_parent.remove_child_node(node)

            # update drivable area and reachable set dictionaries
            self.dict_step_to_drivable_area[step] = [node.position_rectangle
                                                     for idx_node, node in enumerate(list_nodes)
                                                     if idx_node not in list_idx_nodes_to_be_deleted]
            self.dict_step_to_reachable_set[step] = [node for idx_node, node in enumerate(list_nodes)
                                                     if idx_node not in list_idx_nodes_to_be_deleted]

            cnt_nodes_after_pruning += len(self.dict_step_to_reachable_set[step])

        self._pruned = True

        util_logger.print_and_log_info(logger, f"\t#Nodes before pruning: \t{cnt_nodes_before_pruning}")
        util_logger.print_and_log_info(logger, f"\t#Nodes after pruning: \t{cnt_nodes_after_pruning}")
