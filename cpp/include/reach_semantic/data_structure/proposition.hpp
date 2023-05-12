#pragma once

#include "reach_semantic/utility/shared_include.hpp"

namespace semantic_reach {
/// Proposition groups.
enum class PropositionGroup {
    POSITION,
    VELOCITY,
    ACCELERATION,
    VEHICLE,
    TRAFFIC_SIGN,
    TRAFFIC_LIGHT,
    INTERSECTION,
    PRIORITY,
    TRAFFIC_STATUS,
    TEMPORARY
};

// TODO: bind this to Python?
class Proposition {
public:
    static inline std::string in_lanelet() {
        return "InLanelet";
    }
    static inline std::string in_lanelet(int const& lanelet_id) {
        return in_lanelet() + "_" + std::to_string(lanelet_id);
    }
    static inline std::string lanelet_transition(int const& lanelet_id_from, int const& lanelet_id_to) {
        return "Lanelet_" + std::to_string(lanelet_id_from) + "_to_" + std::to_string(lanelet_id_to);
    }
};
}