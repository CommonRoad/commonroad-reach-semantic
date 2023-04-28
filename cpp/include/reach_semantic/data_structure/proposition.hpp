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
}