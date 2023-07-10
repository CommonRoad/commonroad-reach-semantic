import time
from abc import ABC

from commonroad_reach.data_structure.collision_checker import CollisionChecker
from commonroad_reach.data_structure.reach.reach_set import ReachableSet

from commonroad_reach_semantic.benchmark.benchmark_result import ReachBenchmarkResults, ReachComputationTimes
from commonroad_reach_semantic.data_structure.config.semantic_configuration import SemanticConfiguration
from commonroad_reach_semantic.data_structure.environment_model.semantic_model import SemanticModel
from commonroad_reach_semantic.data_structure.rule.traffic_rule_interface import TrafficRuleInterface


class SemanticReachableSet(ReachableSet, ABC):
    benchmark_result: ReachBenchmarkResults

    def __init__(self, config: SemanticConfiguration, semantic_model: SemanticModel,
                 rule_interface: TrafficRuleInterface):
        super().__init__(config)
        self.benchmark_result = ReachBenchmarkResults()
        time_start = time.perf_counter()
        self.rule_interface = rule_interface
        self.collision_checker = CollisionChecker(self.config)
        self.benchmark_result.other_initialization_time += time.perf_counter() - time_start
        for step in range(self.step_start, self.step_end + 1):
            self.benchmark_result.computation_times_per_step[step] = ReachComputationTimes()
