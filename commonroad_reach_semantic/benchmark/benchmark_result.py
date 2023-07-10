from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict


@dataclass
class ReachBenchmarkResults:
    """Class to store the results of a benchmark run."""
    computation_times_per_step: Dict[int, ReachComputationTimes] = field(default_factory=dict)
    automaton_creation_time: float = 0.0
    other_initialization_time: float = 0.0
    pruning_time: float = 0.0
    cnt_nodes_before_pruning: int = 0
    cnt_nodes_after_pruning: int = 0

    @property
    def total_time(self) -> float:
        return self.automaton_creation_time + self.other_initialization_time + \
            sum(step_times.total for step_times in self.computation_times_per_step.values()) + \
            self.pruning_time

    @property
    def total_propagation_time(self) -> float:
        return sum(step_times.propagation for step_times in self.computation_times_per_step.values())

    @property
    def total_splitting_time(self) -> float:
        return sum(step_times.splitting for step_times in self.computation_times_per_step.values())

    @property
    def total_partitioning_time(self) -> float:
        return sum(step_times.partitioning for step_times in self.computation_times_per_step.values())

    @property
    def total_collision_check_time(self) -> float:
        return sum(step_times.collision_check for step_times in self.computation_times_per_step.values())

    @property
    def total_merge_time(self) -> float:
        return sum(step_times.merge for step_times in self.computation_times_per_step.values())

    @property
    def total_node_creation_time(self) -> float:
        return sum(step_times.node_creation for step_times in self.computation_times_per_step.values())

    def __str__(self) -> str:
        return "\n".join([
            f"==============Results================",
            f"Nodes before pruning:\t{self.cnt_nodes_before_pruning}",
            f"Nodes after pruning:\t{self.cnt_nodes_after_pruning}",
            f"-------------------------------------",
            f"Automaton creation time:\t{self.automaton_creation_time:.3f}s",
            f"Other initialization time:\t{self.other_initialization_time:.3f}s",
            f"Propagation time:\t\t\t{self.total_propagation_time:.3f}s",
            f"Splitting time:\t\t\t\t{self.total_splitting_time:.3f}s",
            f"Partitioning time:\t\t\t{self.total_partitioning_time:.3f}s",
            f"Collision check time:\t\t{self.total_collision_check_time:.3f}s",
            f"Merge time:\t\t\t\t\t{self.total_merge_time:.3f}s",
            f"Node creation time:\t\t\t{self.total_node_creation_time:.3f}s",
            f"Pruning time:\t\t\t\t{self.pruning_time:.3f}s",
            f"-------------------------------------",
            f"Total time:\t\t\t\t\t{self.total_time:.3f}s",
            f"=====================================",
        ])


@dataclass
class ReachComputationTimes:
    """Class to store the computation times of a single reach set step."""
    propagation: float = 0.0
    splitting: float = 0.0
    partitioning: float = 0.0
    collision_check: float = 0.0
    merge: float = 0.0
    node_creation: float = 0.0

    @property
    def total(self) -> float:
        return self.propagation + self.splitting + self.partitioning + self.collision_check + self.merge + self.node_creation
