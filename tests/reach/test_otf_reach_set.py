from commonroad_reach_semantic.data_structure.reach.semantic_otf_reach_set_py import PySemanticOTFReachableSet


class TestOTFReachSet:

    def test_overall(self, semantic_otf_reachable_set_py: PySemanticOTFReachableSet):
        """Checks if any exceptions occur during reachable set computation."""
        step_start = semantic_otf_reachable_set_py.step_start + 1
        step_end = semantic_otf_reachable_set_py.step_end
        semantic_otf_reachable_set_py.compute(step_start, step_end)
