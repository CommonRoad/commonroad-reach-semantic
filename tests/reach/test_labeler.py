from commonroad_reach.data_structure.reach.reach_node import ReachNode
from commonroad_reach.data_structure.reach.reach_polygon import ReachPolygon
from commonroad_reach.utility import reach_operation

from commonroad_reach_semantic.data_structure.reach.reachable_set_labeler import ReachableSetLabeler
from commonroad_reach_semantic.data_structure.rule.proposition import Proposition as Prop


class TestLabeler:

    def test_label_initial_state(self, reachable_set_labeler: ReachableSetLabeler):
        config = reachable_set_labeler.semantic_model.config
        tuple_vertices_polygon_lon, tuple_vertices_polygon_lat = \
            reach_operation.generate_tuples_vertices_polygons_initial(config)

        polygon_lon = ReachPolygon.from_rectangle_vertices(*tuple_vertices_polygon_lon)
        polygon_lat = ReachPolygon.from_rectangle_vertices(*tuple_vertices_polygon_lat)

        initial_state = ReachNode(polygon_lon, polygon_lat, config.planning.step_start)

        reachable_set_labeler.label_initial_state([initial_state], config.planning.step_start)

        initial_propsitions = reachable_set_labeler.reachable_set_to_propositions[initial_state].set_propositions
        initial_lanelet_ids = reachable_set_labeler.reachable_set_to_lanelet_ids[initial_state]

        # check that correct lanelet and relation to obstacle are present
        assert initial_propsitions.issuperset({
            Prop.in_lanelet(1),
            Prop.behind(8), Prop.left_of(8)
        })
        # check that only the correct lanelet and relation to obstacle is present
        assert initial_propsitions.isdisjoint({
            Prop.in_lanelet(2), Prop.in_lanelet(3), Prop.in_lanelet(4),
            Prop.beside(8), Prop.in_front_of(8), Prop.aligned_with(8), Prop.right_of(8)
        })
        # check that there are no labels for bogus lanelets or obstacles
        assert initial_propsitions.isdisjoint({
            Prop.in_lanelet(0), Prop.in_lanelet(-1), Prop.in_lanelet(6), Prop.in_lanelet(9000),
            Prop.behind(0), Prop.behind(-1), Prop.behind(6), Prop.behind(9000),
            Prop.left_of(0), Prop.left_of(-1), Prop.left_of(6), Prop.left_of(9000),
            Prop.right_of(0), Prop.right_of(-1), Prop.right_of(6), Prop.right_of(9000),
        })
        # check that lanelet ids matches propositions
        assert initial_lanelet_ids == {1}
