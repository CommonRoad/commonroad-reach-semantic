#pragma once

#include <map>

namespace semantic_reach {
    using Microseconds = long;

    struct ReachComputationTimes {
        Microseconds propagation{};
        Microseconds splitting{};
        Microseconds partitioning{};
        Microseconds collision_check{};
        Microseconds merge{};
        Microseconds node_creation{};
    };

    struct ReachBenchmarkResults {
        std::map<int, ReachComputationTimes> computation_times_per_step{};
        Microseconds automaton_creation_time{};
        int cnt_nodes_before_pruning{};
        int cnt_nodes_after_pruning{};
    };
}
