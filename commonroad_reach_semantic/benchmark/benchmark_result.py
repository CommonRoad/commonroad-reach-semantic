from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict


@dataclass
class ReachBenchmarkResults:
    """Class to store the results of a benchmark run."""
    computation_times_per_step: Dict[int, ReachComputationTimes] = field(default_factory=dict)
    automaton_creation_time: float = 0.0
    pruning_time: float = 0.0
    cnt_nodes_before_pruning: int = 0
    cnt_nodes_after_pruning: int = 0

    @property
    def total_time(self) -> float:
        return self.automaton_creation_time + \
            sum(step_times.total for step_times in self.computation_times_per_step.values()) + \
            self.pruning_time

    @property
    def total_propagation_time(self):
        return sum(step_times.propagation for step_times in self.computation_times_per_step.values())

    @property
    def total_splitting_time(self):
        return sum(step_times.splitting for step_times in self.computation_times_per_step.values())

    @property
    def total_collision_check_time(self):
        return sum(step_times.collision_check for step_times in self.computation_times_per_step.values())

    @property
    def total_node_creation_time(self):
        return sum(step_times.node_creation for step_times in self.computation_times_per_step.values())


@dataclass
class ReachComputationTimes:
    """Class to store the computation times of a single reach set step."""
    propagation: float = 0.0
    splitting: float = 0.0
    collision_check: float = 0.0
    node_creation: float = 0.0

    @property
    def total(self) -> float:
        return self.propagation + self.splitting + self.collision_check + self.node_creation
